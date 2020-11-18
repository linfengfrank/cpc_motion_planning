#include <ros/ros.h>
#include <mid_plan/utils/a_star.h>
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
#include "cpc_motion_planning/astar_service.h"

#define SHOWPC

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* point_pub;
ros::Publisher* pc_pub;
Astar *mid_map=nullptr;
bool received_map = false;
bool received_ref = false;
cpc_motion_planning::ref_data ref;

//---
void publishMap(const std::vector<CUDA_GEO::pos> &path)
{
  //publish the point cloud to rviz for checking
  CUDA_GEO::pos p;
  for (int i=0; i<path.size(); i++)
  {
    p = path[i];
    pcl::PointXYZRGB clrP;
    clrP.x = p.x;
    clrP.y = p.y;
    clrP.z = p.z;
    clrP.a = 255;
    clrP.r = 255;
    pclOut->points.push_back (clrP);
  }
  pc_pub->publish(pclOut);
  pclOut->clear();
}
//---
float actualDistBetweenCoords(const CUDA_GEO::coord &c1, const CUDA_GEO::coord &c2)
{
  CUDA_GEO::coord c = c1-c2;
  return mid_map->getGridStep()*sqrt(static_cast<float>(c.square()));
}
//---
float planPath(const CUDA_GEO::coord &start, const CUDA_GEO::coord &goal, std::vector<CUDA_GEO::coord> &path,
                const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_value_map=nullptr)
{
  float length = 0;
  path = mid_map->AStar2D(goal,start,false,length,crd_shift,last_value_map);
  length += 1*actualDistBetweenCoords(path[0],goal);
  return length;
}
//---
float pathLength(const std::vector<CUDA_GEO::coord> &path)
{
  float length = 0;
  CUDA_GEO::coord shift;
  for (int i=0; i<path.size()-1;i++)
  {
    shift = path[i+1]-path[i];
    length += sqrt(static_cast<float>(shift.square()))*mid_map->getGridStep();
  }
  return length;
}
//---
float cascadePath(const std::vector<CUDA_GEO::coord> &path, std::vector<CUDA_GEO::coord> &cascade, CUDA_GEO::coord goal)
{
  planPath(path[0], goal, cascade);
  cascade.insert(cascade.end(),path.begin(),path.end());
  return pathLength(cascade) + 1*actualDistBetweenCoords(cascade[0],goal);
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  CUDA_GEO::pos origin;
  if (mid_map == nullptr)
  {
    mid_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
  }
  mid_map->copyEdtData(msg);
}
//---
bool mid_plan(cpc_motion_planning::astar_service::Request &req,
              cpc_motion_planning::astar_service::Response &res)
{
  if (!received_map)
    return false;

  auto start_time = std::chrono::steady_clock::now();

  // Set all points
  int tgt_height_coord = mid_map->calcTgtHeightCoord(0.0f);

  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);
  start.z = tgt_height_coord;

  CUDA_GEO::pos goal;
  goal.x = req.target.position.x;
  goal.y = req.target.position.y;
  goal.z = req.target.position.z;

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

  // Do the planning
  // Initialize the list
  std::vector<CUDA_GEO::coord> path_adopt;

  if (!received_ref)
  {
    // Plan A: the new plan
    planPath(start,glb_tgt,path_adopt);
  }
  else
  {
    // Plan B: cascade based on current trajectory
    std::vector<CUDA_GEO::coord> path_tmp;
    float speed = fabsf(ref.data[0]);
    int max_i = max(static_cast<int>(speed/0.5f/0.05f),20);
    std::cout<<"----"<<max_i<<"----"<<ref.cols<<std::endl;
    for (int i=0; i<ref.cols; i++)
    {
      CUDA_GEO::pos p(ref.data[i*ref.rows+3],ref.data[i*ref.rows+4],0);
      CUDA_GEO::coord c = mid_map->pos2coord(p);
      c.z = tgt_height_coord;

      if (i > max_i)
        break;

      if (path_tmp.empty() || !(path_tmp.back() == c))
      {
        if(mid_map->isOccupied(c) || !mid_map->isInside(c))
          break;
        else
          path_tmp.push_back(c);
      }
    }

    if (path_tmp.empty())
    {
      path_tmp.push_back(start);
    }

    std::reverse(path_tmp.begin(),path_tmp.end());
    cascadePath(path_tmp,path_adopt,glb_tgt);
  }

  // The function findSplitCoords will reverse the path sequence
  std::vector<CUDA_GEO::pos>path = mid_map->findSplitCoords(path_adopt);
  publishMap(path);

  geometry_msgs::Pose tmp_pose;
  for (CUDA_GEO::pos p : path)
  {
    tmp_pose.position.x = p.x;
    tmp_pose.position.y = p.y;
    tmp_pose.position.z = p.z;
    res.wps.push_back(tmp_pose);
  }

  // Check whether the true goal is reachable or not
  res.reachable = true;
  if(mid_map->isInside(mid_map->pos2coord(goal)) && sqrtf((path.back()-goal).square()) > 2*mid_map->getGridStep())
  {
    // The target is inside the map and is not reached by the A* path
    res.reachable = false;
  }

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "A-star planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;

  return true;
}
//---
void get_reference(const cpc_motion_planning::ref_data::ConstPtr &msg)
{
  received_ref = true;
  ref = *msg;
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  pc_pub = new ros::Publisher;
  point_pub = new ros::Publisher;

  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/lowres_map", 1, &mapCallback);
  ros::ServiceServer service = nh.advertiseService("/astar_service", mid_plan);

  ros::Subscriber state_sub = nh.subscribe("/ref_traj", 1, get_reference);


  *pc_pub = nh.advertise<PointCloud> ("/path", 1);
  *point_pub = nh.advertise<geometry_msgs::PoseStamped>("/mid_layer/",1);
  pclOut->header.frame_id = "/world";

  ros::spin();

  delete pc_pub;
  delete point_pub;

  if (mid_map)
  {
    delete mid_map;
  }
  return 0;
}
