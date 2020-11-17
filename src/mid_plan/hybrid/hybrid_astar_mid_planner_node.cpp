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
#include <mid_plan/hybrid/hybrid_astar.h>
#include <mid_plan/a_star.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/smooth_plan_request.h>
#include <cpc_motion_planning/collision_check.h>
#define SHOWPC
//#define DEBUG_COST
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* path_vis_pub;
ros::Publisher* path_pub;
HybridAstar *ha_map=nullptr;

bool received_map = false;
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  if (ha_map == nullptr)
  {
    ha_map = new HybridAstar(msg->x_size,msg->y_size,msg->z_size);
    ha_map->copyEdtData(msg);
    ha_map->set_neighbours();
  }
  else
  {
    ha_map->copyEdtData(msg);
  }
}
//---
void showPath(const cpc_motion_planning::path &path_cell)
{
  for (size_t i=0;i<path_cell.actions.size();i++)
  {
    cpc_motion_planning::path_action pa = path_cell.actions[i];
    for (size_t j=0; j<pa.x.size(); j++)
    {
      pcl::PointXYZRGB clrP;
      clrP.x = pa.x[j];
      clrP.y = pa.y[j];
      clrP.z = pa.theta[j];
      clrP.r = 250*(i%3==0);
      clrP.g = 250*(i%3==1);
      clrP.b = 250*(i%3==2);

      if (pa.type == 1)
      {
        clrP.r = 250;
        clrP.g = 250;
        clrP.b = 250;
      }
      pclOut->points.push_back (clrP);
    }
  }

  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);
  path_vis_pub->publish (pclOut);
  pclOut->clear();
}
//---
void glb_plan(const cpc_motion_planning::plan_request::ConstPtr &msg)
{
  if (!received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

#ifdef DEBUG_COST
  static bool done = false;
  if (done) return;
  done = true;
#endif

  float3 start_pose = make_float3(msg->start_x,msg->start_y,msg->start_theta);
  CUDA_GEO::pos goal_pos(msg->goal_x,msg->goal_y,0);
  CUDA_GEO::coord goal_coord = ha_map->pos2coord(goal_pos);
  goal_coord.z = 0;

  std::vector<HybridAstar::path_info> path = ha_map->plan(start_pose,goal_coord);
  cpc_motion_planning::path path_cell = ha_map->split_path(path);
  path_cell.request_ctt = msg->request_ctt;
  path_pub->publish(path_cell);


#ifdef SHOWPC
  showPath(path_cell);
#endif



  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Recover planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;
}

//---
void smooth_glb_plan(const cpc_motion_planning::smooth_plan_request::ConstPtr &msg)
{
  if (!received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

#ifdef DEBUG_COST
  static bool done = false;
  if (done) return;
  done = true;
#endif

  float3 start_pose = make_float3(msg->start_x,msg->start_y,msg->start_theta);
  std::vector<HybridAstar::path_info> pre_path;
  HybridAstar::path_info pre_pi;
  size_t i_max = min(msg->current_ref_path.x.size(),20);
  for(size_t i = 0; i<i_max; i++)
  {
    float3 pose = make_float3(msg->current_ref_path.x[i],
                              msg->current_ref_path.y[i],
                              msg->current_ref_path.theta[i]);
    if (ha_map->hybrid_isfree(pose))
    {
      start_pose=pose;
      pre_pi.pose = pose;
      pre_pi.action = make_float3(msg->current_ref_path.w[i],
                                  msg->current_ref_path.v[i],
                                  msg->current_ref_path.dt[i]);
      pre_path.push_back(pre_pi);
    }
    else
    {
      break;
    }
  }



  CUDA_GEO::pos goal_pos(msg->goal_x,msg->goal_y,0);
  CUDA_GEO::coord goal_coord = ha_map->pos2coord(goal_pos);
  goal_coord.z = 0;

  std::vector<HybridAstar::path_info> path = ha_map->plan(start_pose,goal_coord);

  for (size_t i=0; i<path.size(); i++)
  {
    pre_path.push_back(path[i]);
  }
  cpc_motion_planning::path path_cell = ha_map->split_path(pre_path);
  path_cell.request_ctt = msg->request_ctt;
  path_pub->publish(path_cell);


#ifdef SHOWPC
  showPath(path_cell);
#endif



  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Recover planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;
}

bool collision_check(cpc_motion_planning::collision_check::Request &req,
                     cpc_motion_planning::collision_check::Response &res)
{
  res.collision = false;
  for(size_t i = 0; i<req.collision_checking_path.x.size(); i++)
  {
    float3 pose = make_float3(req.collision_checking_path.x[i],
                              req.collision_checking_path.y[i],
                              req.collision_checking_path.theta[i]);
    if (!ha_map->hybrid_isfree(pose))
    {
      res.collision = true;
      break;
    }
  }
  return true;
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  path_vis_pub = new ros::Publisher;
  path_pub = new ros::Publisher;

  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber mid_plan_sub = nh.subscribe("/plan_request", 1, &glb_plan);
  ros::Subscriber mid_smooth_plan_sub = nh.subscribe("/smooth_plan_request", 1, &smooth_glb_plan);
  ros::ServiceServer service = nh.advertiseService("/collision_check", collision_check);

  *path_vis_pub = nh.advertise<PointCloud> ("/path_vis", 1);
  *path_pub = nh.advertise<cpc_motion_planning::path> ("/hybrid_path", 1);

  pclOut->header.frame_id = "/world";


  ros::spin();

  delete path_vis_pub;
  delete path_pub;

  if (ha_map)
  {
    delete ha_map;
  }
  return 0;
}
