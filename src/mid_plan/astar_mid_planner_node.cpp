#include <ros/ros.h>
#include <mid_plan/a_star.h>
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
#define USE2D
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
#ifdef USE2D
  path = mid_map->AStar2D(goal,start,false,length,crd_shift,last_value_map);
#else
  path = mid_map->AStar3D(goal,start,false,length,crd_shift,last_value_map);
#endif
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
#ifdef USE2D
  start.z = tgt_height_coord;
#endif

  CUDA_GEO::pos goal;
  goal.x = req.target.position.x;
  goal.y = req.target.position.y;
  goal.z = req.target.position.z;

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

  // Do the planning
  // Initialize the list
  std::vector<std::vector<CUDA_GEO::coord>> path_list(3);
  std::vector<float> cost_list(2);
  std::vector<CUDA_GEO::coord> path_adopt;

  if (1)
  {
    // Plan A: the new plan
    cost_list[0]=planPath(start,glb_tgt,path_list[0]);
  }
  else
  {
    // Plan B: modify based on current trajectory
    float ini_v_x = ref.data[3];
    float ini_v_y = ref.data[4];
    float ini_v_z = ref.data[5];
    float speed = sqrtf(ini_v_x*ini_v_x + ini_v_y*ini_v_y + ini_v_z*ini_v_z);
    int max_i = max(speed/0.8f/0.05f,20);
    //std::cout<<"----"<<max_i<<"----"<<ref.cols<<std::endl;
    for (int i=0; i<ref.cols; i++)
    {
      CUDA_GEO::pos p(ref.data[i*ref.rows],ref.data[i*ref.rows+1],ref.data[i*ref.rows+2]);
      CUDA_GEO::coord c = mid_map->pos2coord(p);
#ifdef USE2D
      c.z = tgt_height_coord;
#endif
      if (i > max_i)
        break;

      if (path_list[1].empty() || !(path_list[1].back() == c))
      {
        if(mid_map->isOccupied(c,0.5f) || !mid_map->isInside(c))
          break;
        else
          path_list[1].push_back(c);
      }
    }

    if (path_list[1].empty())
    {
      path_list[1].push_back(start);
    }

    std::reverse(path_list[1].begin(),path_list[1].end());
    cost_list[1] = cascadePath(path_list[1],path_list[2],glb_tgt);
  }

  unsigned int selected_case = 0;
  if(1)
  {
    path_adopt = path_list[0];
    selected_case = 0;
  }
  else
  {
    selected_case = 2;
    path_adopt = path_list[2];
  }

  //std::reverse(path_adopt.begin(),path_adopt.end());
  std::vector<CUDA_GEO::pos>path = mid_map->findSplitCoords(path_adopt);
  publishMap(path);
  //publish the guide line
//  std::reverse(path_adopt.begin(),path_adopt.end());
//  cpc_motion_planning::guide_line line_msg;
//  geometry_msgs::Point line_pnt;
//  guide_path.clear();
//  for (CUDA_GEO::coord &path_crd : path_adopt)
//  {
//    CUDA_GEO::pos path_pnt = mid_map->coord2pos(path_crd);
//    guide_path.push_back(path_pnt);
//    line_pnt.x = path_pnt.x;
//    line_pnt.y = path_pnt.y;
//    line_pnt.z = path_pnt.z;
//    line_msg.pts.push_back(line_pnt);
//  }
//  line_pub->publish(line_msg);


  geometry_msgs::Pose tmp_pose;
  for (CUDA_GEO::pos p : path)
  {
    tmp_pose.position.x = p.x;
    tmp_pose.position.y = p.y;
    tmp_pose.position.z = p.z;
    res.wps.push_back(tmp_pose);
  }
  //std::cout<<res.wps.size()<<std::endl;

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;

      return true;
  //extract_target(msg);
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
