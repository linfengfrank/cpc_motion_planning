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
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#define SHOWPC


  tf::Vector3 gt_offset;
  bool got_offset=false;

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
bool m_pose_received = false;
nav_msgs::Odometry m_pose;
//---
//publish the NF1 map as point cloud to rviz for checking
void publishMap()
{
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  for (int x=0;x<mid_map->getMaxX();x++)
  {
      for (int y=0;y<mid_map->getMaxY();y++)
      {
          for (int z=0;z<mid_map->getMaxZ();z++)
          {
              // Construct the coordinate
              c.x = x;
              c.y = y;
              c.z = z;

              // Get the NF1 value
              float d_c = mid_map->getCost2Come(c,0.0f)*5;


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
    msg.z_size = map->getMaxZ();
    msg.payload8.resize(sizeof(float)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map)
{
  CUDA_GEO::coord c;
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int z=0;z<map->getMaxZ();z++)
  {
      for (int y=0;y<map->getMaxY();y++)
      {
          for (int x=0;x<map->getMaxX();x++)
          {

              c.x = x;
              c.y = y;
              c.z = z;
              float d_c = mid_map->getCost2Come(c,0.0);
              tmp[i++]=d_c;
          }
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
void taskGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  received_cmd = true;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = msg->pose.position.z;
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
void get_gt_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(got_offset)
  {
    return;
  }
  gt_offset[0]=msg->pose.pose.position.x;
  gt_offset[1]=msg->pose.pose.position.y;
  gt_offset[2]=msg->pose.pose.position.z;
  got_offset=true;
  std::cout<<"---------------------offset x is "<<gt_offset[0]<<std::endl;
}
//---
void glb_plan(const ros::TimerEvent&)
{
  if (!received_cmd || !received_map || !m_pose_received)
    return;

  auto start_time = std::chrono::steady_clock::now();


  // Calculate the start point coordinate
  CUDA_GEO::pos start_pos(m_pose.pose.pose.position.x,
                          m_pose.pose.pose.position.y,
                          m_pose.pose.pose.position.z);

  CUDA_GEO::coord start = mid_map->pos2coord(start_pos);
  start = mid_map->get_first_free_coord(start,mid_safety_radius);

  // Calculate the goal point coordinate
  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  // Put the target inside the square box of the Dijkstra map
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

  // Find the available target which is path[0] (the last point on the Astar path)
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = a_map->AStar3D(glb_tgt,start,false,length,mid_safety_radius);

  // Do dijkstra search with path[0] as the target
  mid_map->dijkstra3D(path[0],mid_safety_radius);

  // Set up the map message and copy the NF1 data
  setup_map_msg(nf1_map_msg,mid_map,false);
  copy_map_to_msg(nf1_map_msg,mid_map);

  // Set and copy the carrot data
  CUDA_GEO::pos carrot = a_map->coord2pos(path[0]);
  nf1_map_msg.x_carrot = carrot.x;
  nf1_map_msg.y_carrot = carrot.y;
  nf1_map_msg.z_carrot = carrot.z;

  // Publish the NF1 map (for local planner)
  nf1_pub->publish(nf1_map_msg);

#ifdef SHOWPC
  geometry_msgs::PoseStamped mid_goal_pose;
  mid_goal_pose.header.frame_id="map";
  mid_goal_pose.pose.position.x = carrot.x;
  mid_goal_pose.pose.position.y = carrot.y;
  mid_goal_pose.pose.position.z = carrot.z;
  mid_goal_pub->publish(mid_goal_pose);

  publishMap();
#endif

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Middle planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;

}
//---
void vehicle_pose_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  ros::NodeHandle nh;

  // Initialize the publishers
  pc_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  *pc_pub = nh.advertise<PointCloud> ("nf1_vis", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("nf1",1);
  *mid_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("mid_goal", 1);

  // Subscribers
  ros::Subscriber map_sub = nh.subscribe("edt_map", 1, &mapCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
  ros::Subscriber pose_sub = nh.subscribe("/ground_truth/state", 1, &vehicle_pose_call_back);
  ros::Subscriber task_tgt_sub = nh.subscribe("task_planner/goal", 1, &goalCallback);

  // other initilization
  pclOut->header.frame_id = "/map";
  mid_safety_radius = 0.51f;

  // start the timer
  mid_plan_timer = nh.createTimer(ros::Duration(0.6), glb_plan);

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
