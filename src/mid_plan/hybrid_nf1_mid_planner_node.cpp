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
#include <mid_plan/hybrid_dijkstra.h>
#include <mid_plan/a_star.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#define SHOWPC
//#define DEBUG_COST
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* nf1_pub;
ros::Publisher* pc_pub;
ros::Publisher* path_pub;
ros::Publisher* mid_goal_pub;
Dijkstra *mid_map=nullptr;
Astar *a_map=nullptr;

float FLY_HEIGHT;
bool first_run = true;
CUDA_GEO::pos curr_target_pos;
bool stucked = false;
bool received_cmd = false;
bool received_map = false;
bool received_ref = false;
CUDA_GEO::pos goal;
ros::Timer glb_plan_timer;
cpc_motion_planning::ref_data ref;
cpc_aux_mapping::grid_map nf1_map_msg;
float3 curr_pose;
bool recevied_odom = false;
//---
void publishMap(int tgt_height_coord)
{
  //publish the point cloud to rviz for checking
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  for (int x=0;x<mid_map->getMaxX();x++)
  {
    for (int y=0;y<mid_map->getMaxY();y++)
    {
      for (int z=0;z<THETA_GRID_SIZE;z++)
      {
        c.x = x;
        c.y = y;
        c.z = z;
        float d_c = mid_map->m_getCost2Come(c,0.0f)*15;
        if (d_c > 255) d_c = 255;
        int d = static_cast<int>(d_c);

        //                std::cout<<d<<" ";
        //int color = (EDTMap::MAX_RANGE*EDTMap::MAX_RANGE-d)*255/EDTMap::MAX_RANGE/EDTMap::MAX_RANGE;
        //if (d < 4.1)
        {
          p = mid_map->coord2pos(c);
          pcl::PointXYZRGB clrP;
          clrP.x = p.x;
          clrP.y = p.y;
          clrP.z = p.z;

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
  }
  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);

  pc_pub->publish (pclOut);

  pclOut->clear();
}
//---


//---
void stuckCallback(const std_msgs::Bool::ConstPtr& msg)
{
  stucked = msg->data;
  std::cout<<"Stucked."<<std::endl;
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
    msg.z_size = THETA_GRID_SIZE;
    msg.payload8.resize(sizeof(float)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map,int tgt_height_coord)
{
  CUDA_GEO::coord c;
#ifdef DEBUG_COST
  std::ofstream mylog;
  mylog.open("/home/sp/test/dj.txt");
#endif
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int z=0;z<THETA_GRID_SIZE;z++)
  {
    for (int y=0;y<map->getMaxY();y++)
    {
      for (int x=0;x<map->getMaxX();x++)
      {
        c.x = x;
        c.y = y;
        c.z = z;
        float d_c = mid_map->m_getCost2Come(c,0.0);
        tmp[i++]=d_c;
#ifdef DEBUG_COST
        mylog<<d_c<<" ";
#endif
      }
    }
  }
#ifdef DEBUG_COST
  mylog.close();
#endif
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  if (mid_map == nullptr)
  {
    mid_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    mid_map->copyEdtData(msg);
    mid_map->set_neighbours();
    setup_map_msg(nf1_map_msg,mid_map,true);

    a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
    a_map->copyEdtData(msg);
  }
  else
  {
    mid_map->copyEdtData(msg);
    a_map->copyEdtData(msg);
  }
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
  if (!received_cmd || !received_map || !recevied_odom)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // Set all points
  int tgt_height_coord = mid_map->calcTgtHeightCoord(FLY_HEIGHT);

  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);
  start.z = tgt_height_coord;

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

#ifdef DEBUG_COST
  static bool done = false;
  if (done) return;
  done = true;
#endif

  float length = 0.0f;
  CUDA_GEO::coord local_target = mid_map->hybrid_bfs(curr_pose,glb_tgt);

//  mid_map->dijkstra_with_theta(path[0]);
  mid_map->hybrid_dijkstra_with_int_theta(local_target);

  setup_map_msg(nf1_map_msg,mid_map,false);
  copy_map_to_msg(nf1_map_msg,mid_map,tgt_height_coord);
  nf1_pub->publish(nf1_map_msg);

  //Get the mid goal
  //std::reverse(path.begin(),path.end());
//  unsigned int tgt_idx = a_map->findTargetCoord(path);
  geometry_msgs::PoseStamped mid_goal_pose;
  CUDA_GEO::pos mid_goal_pos = mid_map->coord2pos(local_target);
  mid_goal_pose.header.frame_id="world";
  mid_goal_pose.pose.position.x = mid_goal_pos.x;
  mid_goal_pose.pose.position.y = mid_goal_pos.y;
  mid_goal_pose.pose.position.z = mid_goal_pos.z;
  mid_goal_pub->publish(mid_goal_pose);

#ifdef SHOWPC
  publishMap(tgt_height_coord);
  std::vector<float3> shortest_path = mid_map->get_shortest_path(curr_pose);
//  std::cout<<"****"<<shortest_path.size()<<std::endl;
  for (size_t i=0;i<shortest_path.size();i++)
  {
    pcl::PointXYZRGB clrP;
    clrP.x = shortest_path[i].x;
    clrP.y = shortest_path[i].y;
    clrP.z = 0;
    pclOut->points.push_back (clrP);
  }

  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);

  path_pub->publish (pclOut);

  pclOut->clear();

#endif


  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Middle planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;

  first_run = false;
}
//---
float get_heading(const nav_msgs::Odometry &odom)
{
  double phi,theta,psi;
  tf::Quaternion q(odom.pose.pose.orientation.x,
                   odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z,
                   odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  return psi;
}
//---
void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  recevied_odom = true;
  curr_pose.x = msg->pose.pose.position.x;
  curr_pose.y = msg->pose.pose.position.y;
  curr_pose.z = get_heading(*msg);
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  pc_pub = new ros::Publisher;
  path_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
  ros::Subscriber slam_odom_sub = nh.subscribe("/UgvOdomTopic", 1, &slam_odo_call_back);

  *pc_pub = nh.advertise<PointCloud> ("/nf1", 1);
  *path_pub = nh.advertise<PointCloud> ("/path", 1);
  *mid_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("/mid_layer/goal",1);

  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,0.0);


  glb_plan_timer = nh.createTimer(ros::Duration(2.0), glb_plan);

  ros::spin();

  delete pc_pub;
  delete nf1_pub;
  delete path_pub;

  if (mid_map)
  {
    delete mid_map;
    delete a_map;
    delete mid_goal_pub;
  }
  return 0;
}
