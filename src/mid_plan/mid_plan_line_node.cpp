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
#include "cpc_motion_planning/guide_line.h"
#define SHOWPC
#define USE2D
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* line_pub;
ros::Publisher* pc_pub;
ros::Publisher* mid_goal_pub;
Astar *mid_map=nullptr;
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
void publishMap(const std::vector<CUDA_GEO::coord> &path, bool blue)
{
  //publish the point cloud to rviz for checking
  CUDA_GEO::pos p;
  for (int i=0; i<path.size(); i++)
  {
    p = mid_map->coord2pos(path[i]);
    pcl::PointXYZRGB clrP;
    clrP.x = p.x;
    clrP.y = p.y;
    clrP.z = p.z;
    clrP.a = 255;
    if (blue)
      clrP.b = 255;
    else
      clrP.g = 255;

    pclOut->points.push_back (clrP);
  }

  //pclOut->clear();
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
void stuckCallback(const std_msgs::Bool::ConstPtr& msg)
{
  stucked = msg->data;
  std::cout<<"Stucked."<<std::endl;
}
//---
CUDA_GEO::coord calculateMapCoordShift(CUDA_GEO::pos old_origin, CUDA_GEO::pos new_origin, float stepSize)
{
  CUDA_GEO::coord output;
  output.x = floor((new_origin.x-old_origin.x)/stepSize + 0.5);
  output.y = floor((new_origin.y-old_origin.y)/stepSize + 0.5);
  output.z = floor((new_origin.z-old_origin.z)/stepSize + 0.5);
  return output;
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
float modifyPath(const std::vector<CUDA_GEO::coord> &path_in, std::vector<CUDA_GEO::coord> &path_out)
{
  // Find the last point with no intersection with obstacle
  int valid_pt = path_in.size()-1; // The last one if there is no intersection
  bool occupied = false;
  for (int i = path_in.size()-1; i>=0; i--)
  {
    mid_map->obsCostAt(path_in[i],0,occupied);
    if (!occupied)
      valid_pt = i;
    else
      break;
  }

  // if valid_pt == 0 it means, the whole path is obstacle free
  if (valid_pt == 0)
  {
    path_out = path_in;
  }
  else
  {
    // otherwise we modify the path
    float tmp;
#ifdef USE2D
    path_out = mid_map->AStar2D(path_in[0],path_in[valid_pt],false,tmp);
#else
    path_out = mid_map->AStar3D(path_in[0],path_in[valid_pt],false,tmp);
#endif

    // cascade the path
    for (int i = valid_pt + 1; i < path_in.size(); i++)
    {
      path_out.push_back(path_in[i]);
    }
  }

  // calculate the length
  return pathLength(path_out) + 1*actualDistBetweenCoords(path_out[0],path_in[0]);
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

  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);
#ifdef USE2D
  start.z = tgt_height_coord;
#endif

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt,8.0f).back();



  // Do the planning
  // Initialize the list
  std::vector<std::vector<CUDA_GEO::coord>> path_list(3);
  std::vector<float> cost_list(2);
  std::vector<CUDA_GEO::coord> path_adopt;

  // Plan A: the new plan
  cost_list[0]=planPath(start,glb_tgt,path_list[0]);

  // Plan B: modify based on current trajectory
  if (received_ref)
  {
    float ini_v_x = ref.data[3];
    float ini_v_y = ref.data[4];
    float ini_v_z = ref.data[5];
    float speed = sqrtf(ini_v_x*ini_v_x + ini_v_y*ini_v_y + ini_v_z*ini_v_z);
    int max_i = max(speed/1.0/0.05,40);
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

  bool np = false;
  bool same_topo = false;
  if(!received_ref)
  {
    path_adopt = path_list[0];
    np = true;
  }
  else
  {
    same_topo = mid_map->checkTopo(path_list[0],path_list[2]);
    float damp_dist= 0.0;
    if (same_topo)
      damp_dist = 1.0;
    else
      damp_dist = 100000.0;

    if (cost_list[1] > cost_list[0] + damp_dist)
    {
      np = true;
      path_adopt = path_list[0];
    }
    else
    {
      np = false;
      path_adopt = path_list[2];
    }
  }


#ifdef SHOWPC
  if (same_topo)
  {
    publishMap(path_list[0],false);
    publishMap(path_list[2],false);
  }
  else
  {
    publishMap(path_list[0],!np);
    publishMap(path_list[2],np);
  }

//  publishMap(path_adopt,curr_tgt,false);
  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);
  pc_pub->publish (pclOut);
  pclOut->clear();

  CUDA_GEO::pos mid_tgt_pos = mid_map->coord2pos(glb_tgt);
  pcl::PointXYZRGB clrP;
  clrP.x = mid_tgt_pos.x;
  clrP.y = mid_tgt_pos.y;
  clrP.z = mid_tgt_pos.z;
  clrP.a = 255;
  clrP.r = 255;
  pclOut->points.push_back (clrP);
  mid_goal_pub->publish(pclOut);
#endif

  std::vector<CUDA_GEO::pos> output_path = mid_map->findSplitCoords(path_adopt);
  cpc_motion_planning::guide_line line_msg;
  geometry_msgs::Point line_pnt;

  for (CUDA_GEO::pos &path_pnt : output_path)
  {
    line_pnt.x = path_pnt.x;
    line_pnt.y = path_pnt.y;
    line_pnt.z = path_pnt.z;
    line_msg.pts.push_back(line_pnt);
  }
  line_pub->publish(line_msg);

  if (stucked)
    stucked = false;

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;

  first_run = false;
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
  line_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
  ros::Subscriber state_sub = nh.subscribe("/ref_traj", 1, get_reference);

  *pc_pub = nh.advertise<PointCloud> ("/path", 1);
  *mid_goal_pub = nh.advertise<PointCloud> ("/mid_goal", 1);
  *line_pub = nh.advertise<cpc_motion_planning::guide_line>("/mid_layer/goal",1);

  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,2.5);


  glb_plan_timer = nh.createTimer(ros::Duration(0.333), glb_plan);

  ros::spin();

  delete pc_pub;
  delete line_pub;

  if (mid_map)
  {
    delete mid_map;
  }
  return 0;
}