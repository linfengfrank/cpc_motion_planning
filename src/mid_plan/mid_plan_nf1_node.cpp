#include <ros/ros.h>
#include <mid_plan/DijkstraMap.h>
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
#define SHOWPC

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* point_pub;
ros::Publisher* pc_pub;
ros::Publisher* mid_goal_pub;
DijkstraMap *mid_map=nullptr;
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
      //for (int z=0;z<msg->z_length;z++)
      {
        c.x = x;
        c.y = y;
        c.z = tgt_height_coord;
        double d_c = mid_map->getCost2Come(c,0.0)*20;
        int d = min(static_cast<int>(d_c),255);

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
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  CUDA_GEO::pos origin;
  if (mid_map == nullptr)
  {
    mid_map = new DijkstraMap(msg->x_size,msg->y_size,msg->z_size);
  }
  mid_map->copyEdtData(msg);
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

  // Set all points
  int tgt_height_coord = mid_map->calcTgtHeightCoord(FLY_HEIGHT);

  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxX()/2);
  start.z = tgt_height_coord;

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt,8.0f).back();

  mid_map->dijkstra2D(glb_tgt);


#ifdef SHOWPC
  publishMap(tgt_height_coord);
#endif
//  geometry_msgs::PoseStamped tgt_msg;
//  tgt_msg.pose.position.x = curr_target_pos.x;
//  tgt_msg.pose.position.y = curr_target_pos.y;
//  tgt_msg.pose.position.z = curr_target_pos.z;
//  point_pub->publish(tgt_msg);

//  if (stucked)
//    stucked = false;

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Middle planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;

  first_run = false;
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  pc_pub = new ros::Publisher;
  point_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);

  *pc_pub = nh.advertise<PointCloud> ("/nf1", 1);
  *mid_goal_pub = nh.advertise<PointCloud> ("/mid_goal", 1);
  *point_pub = nh.advertise<geometry_msgs::PoseStamped>("/mid_layer/goal",1);

  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,2.0);


  glb_plan_timer = nh.createTimer(ros::Duration(0.333), glb_plan);

  ros::spin();

  delete pc_pub;
  delete point_pub;

  if (mid_map)
  {
    delete mid_map;
  }
  return 0;
}
