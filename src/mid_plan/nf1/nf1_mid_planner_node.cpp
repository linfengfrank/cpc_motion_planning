#include <ros/ros.h>
#include <mid_plan/grid_graph.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <algorithm>
#include "cpc_motion_planning/ref_data.h"
#include <mid_plan/dijkstra.h>
#include <mid_plan/a_star.h>
#define SHOWPC

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud); //Point cloud to show the NF1 map
ros::Publisher* nf1_pub; // Publisher of NF1 map (for local planner)
ros::Publisher* pc_pub; // Publisher of NF1 map (for visulization)
ros::Publisher* mid_goal_pub; // Publisher of carrot (for visulization)
Dijkstra *mid_map=nullptr; // Dijkstra planner
Astar *a_map=nullptr; // Astar planner

float fly_height; // Desired fly height
bool received_cmd = false; // Flag, whether goal command is received
bool received_map = false; // Flag, whether map is received
CUDA_GEO::pos goal; // The desired goal location
ros::Timer mid_plan_timer; // Main loop timer
cpc_aux_mapping::grid_map nf1_map_msg; // NF1 map that is to be published
float mid_safety_radius;
//---
//publish the NF1 map as point cloud to rviz for checking
void publishMap(int tgt_height_coord)
{
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  for (int x=0;x<mid_map->getMaxX();x++)
  {
    for (int y=0;y<mid_map->getMaxY();y++)
    {
      // Construct the coordinate
      c.x = x;
      c.y = y;
      c.z = tgt_height_coord;

      // Get the NF1 value
      float d_c = mid_map->getCost2Come(c,0.0f)*15;
      if (d_c > 255) d_c = 255;
      int d = static_cast<int>(d_c);

      // Get its position
      p = mid_map->coord2pos(c);
      pcl::PointXYZRGB clrP;
      clrP.x = p.x;
      clrP.y = p.y;
      clrP.z = p.z;

      // Clour the point based on its NF1 value
      if ( d < 128)
      {
        clrP.b = 255-2*static_cast<unsigned char>(d);
        clrP.g = 2*static_cast<unsigned char>(d);
      }
      else
      {
        clrP.g = 255 - 2*(static_cast<unsigned char>(d) - 128);
        clrP.r = 2*(static_cast<unsigned char>(d)-128);
      }

      clrP.a = 255;
      pclOut->points.push_back (clrP);
    }
  }
  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);
  pc_pub->publish (pclOut);
  pclOut->clear();
}
//---
void setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize)
{
  msg.x_origin = map->getOrigin().x;
  msg.y_origin = map->getOrigin().y;
  msg.z_origin = map->getOrigin().z;
  msg.width = map->getGridStep();

  if (resize)
  {
    msg.x_size = map->getMaxX();
    msg.y_size = map->getMaxY();
    // Currently it is a 2D NF1 map, therefore it has only ONE height layer
    msg.z_size = 1;
    msg.payload8.resize(sizeof(float)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map,int tgt_height_coord)
{
  CUDA_GEO::coord c;
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int y=0;y<map->getMaxY();y++)
  {
    for (int x=0;x<map->getMaxX();x++)
    {
      c.x = x;
      c.y = y;
      c.z = tgt_height_coord;
      float d_c = mid_map->getCost2Come(c,0.0);
      tmp[i++]=d_c;
    }
  }
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  CUDA_GEO::pos origin;
  if (mid_map == nullptr)
  {
    // Setup the Dijkstra planner
    mid_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    setup_map_msg(nf1_map_msg,mid_map,true);

    // Setup the A-star planner
    a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
  }
  // Copy EDT data into the planner
  mid_map->copyEdtData(msg);
  a_map->copyEdtData(msg);
}
//---
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  received_cmd = true;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = msg->pose.position.z;
}
//---
void glb_plan(const ros::TimerEvent&)
{
  if (!received_cmd || !received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // If it is 2D map mode (!! Important here !!)
  if (mid_map->getMaxZ() == 1)
  {
    //We set the fly height to the height of the map (by geting height from its origin)
    fly_height = mid_map->getOrigin().z;
  }

  // Find the z coordinate of the desired height in the Dijkstra map
  int tgt_height_coord = mid_map->calcTgtHeightCoord(fly_height);

  // Calculate the start point coordinate
  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);
  start.z = tgt_height_coord; // Put the start point in the desired fly height
  start = mid_map->get_first_free_coord(start,mid_safety_radius);

  // Calculate the goal point coordinate
  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  // Put the target inside the square box of the Dijkstra map
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

  // Find the available target which is path[0] (the last point on the Astar path)
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = a_map->AStar2D(glb_tgt,start,false,length,mid_safety_radius);

  // Do dijkstra search with path[0] as the target
  mid_map->dijkstra2D(path[0],mid_safety_radius);

  // Set up the map message and copy the NF1 data
  setup_map_msg(nf1_map_msg,mid_map,false);
  copy_map_to_msg(nf1_map_msg,mid_map,tgt_height_coord);

  // Set and copy the carrot data
  CUDA_GEO::pos carrot = a_map->coord2pos(path[0]);
  nf1_map_msg.x_carrot = carrot.x;
  nf1_map_msg.y_carrot = carrot.y;
  nf1_map_msg.z_carrot = carrot.z;

  // Publish the NF1 map (for local planner)
  nf1_pub->publish(nf1_map_msg);

#ifdef SHOWPC
  geometry_msgs::PoseStamped mid_goal_pose;
  mid_goal_pose.header.frame_id="world";
  mid_goal_pose.pose.position.x = carrot.x;
  mid_goal_pose.pose.position.y = carrot.y;
  mid_goal_pose.pose.position.z = carrot.z;
  mid_goal_pub->publish(mid_goal_pose);

  publishMap(tgt_height_coord);
#endif

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Middle planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;

}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  ros::NodeHandle nh;

  // Read the parameters
  nh.param<float>("/nndp_cpp/fly_height",fly_height,2.0);

  // Initialize the publishers
  pc_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  *pc_pub = nh.advertise<PointCloud> ("/nf1_vis", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("/nf1",1);
  *mid_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);

  // Subscribers
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);

  // other initilization
  pclOut->header.frame_id = "/world";
  mid_safety_radius = 1.0f;

  // start the timer
  mid_plan_timer = nh.createTimer(ros::Duration(0.333), glb_plan);

  ros::spin();

  // Delete the publishers
  delete pc_pub;
  delete nf1_pub;

  // Delete the planners
  if (mid_map)
  {
    delete mid_map;
    delete a_map;
  }
  return 0;
}
