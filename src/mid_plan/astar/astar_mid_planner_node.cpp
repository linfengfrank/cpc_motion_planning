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
#include "spline_optimizer/Spline.h"
#include "edt/gpu_edt.cuh"
#include "cutt/cutt.h"

#define SHOWPC
#define USE2D

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud); // Point cloud to show the path
ros::Publisher* line_vis_pub; // Publisher to visulize the guidance path
ros::Publisher* astar_vis_pub; // Publisher to visulize the smoothed path
ros::Publisher* line_map_vis_pub; // Publisher to visulize the smoothed path
ros::Publisher* line_pub; // Publisher of the guidance line
Astar *mid_map=nullptr; // A star planner
float _2D_fly_height; // Targeted fly height at 2D case
bool received_cmd = false; // Flag to check whether a target has been received
bool received_map = false; // Flag to check whether a map has been received
bool received_ref = false; // Flag to check wheter the current reference has been received
CUDA_GEO::pos goal; // The global goal
ros::Timer glb_plan_timer; // Timer to trigger the mid planner
Eigen::Matrix3d current_ref_state; // To log down the current state according to the reference
float mid_safety_radius; // Safety radius during A* search
SPL_OPTI::Spline *spl; // Smoothing spline pointer
ceres::Problem problem; // Nonlinear least square problem
ceres::Solver::Options options; // Ceres Solver options
ceres::Solver::Summary summary; // Ceres Solving summary
EDTMap* m_line_map=nullptr; // Distance to line map, used to calcualte the distance go guidance line
cuttHandle m_rot_plan[3]; // Rotation plan
cpc_motion_planning::guide_line line_msg; //line msg to be published
//---
void publish_path(const cpc_motion_planning::guide_line &path)
{
  //publish the path as point cloud to rviz for checking
  for (int i=0; i<path.pts.size(); i++)
  {
    pcl::PointXYZRGB clrP;
    clrP.x = path.pts[i].x;
    clrP.y = path.pts[i].y;
    clrP.z = path.pts[i].z;
    clrP.a = 255;
    pclOut->points.push_back (clrP);
  }
  line_vis_pub->publish(pclOut);
  pclOut->clear();
}
//---
void publish_a_star_path(const std::vector<CUDA_GEO::pos> &path)
{
  //publish the path as point cloud to rviz for checking
  for (int i=0; i<path.size(); i++)
  {
    pcl::PointXYZRGB clrP;
    clrP.x = path[i].x;
    clrP.y = path[i].y;
    clrP.z = path[i].z;
    clrP.a = 255;
    pclOut->points.push_back (clrP);
  }
  astar_vis_pub->publish(pclOut);
  pclOut->clear();
}
//----
void publish_line_map()
{
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  int idx;
  for (c.x=0;c.x<m_line_map->m_map_size.x;c.x++)
  {
    for (c.y=0;c.y<m_line_map->m_map_size.y;c.y++)
    {
      for (c.z=0;c.z<m_line_map->m_map_size.z;c.z++)
      {
        idx = m_line_map->to_id(c.x, c.y, c.z);
        float dist = m_line_map->m_hst_sd_map[idx].d;
        if (dist*m_line_map->m_grid_step <= 0.5)
        {
          p = m_line_map->coord2pos(c);
          pcl::PointXYZRGB clrP;
          clrP.x = p.x;
          clrP.y = p.y;
          clrP.z = p.z;
          pclOut->points.push_back (clrP);
        }
      }
    }
  }
  line_map_vis_pub->publish (pclOut);
  pclOut->clear();
}
//---
void planPath(const CUDA_GEO::coord &start, const CUDA_GEO::coord &goal, std::vector<CUDA_GEO::coord> &path)
{
  float length = 0;
#ifdef USE2D
  path = mid_map->AStar2D(goal,start,false,length,mid_safety_radius);
#else
  path = mid_map->AStar3D(goal,start,false,length,mid_safety_radius);
#endif
}
//---
void setup_map_msg(cpc_aux_mapping::grid_map &msg, EDTMap* map, bool resize)
{
  // Setup size the grid width
  msg.x_origin = map->m_origin.x;
  msg.y_origin = map->m_origin.y;
  msg.z_origin = map->m_origin.z;
  msg.width = map->m_grid_step;

  if (resize)
  {
    msg.x_size = map->m_map_size.x;
    msg.y_size = map->m_map_size.y;
    msg.z_size = map->m_map_size.z;
    msg.payload8.resize(sizeof(SeenDist)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_EDT;
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  CUDA_GEO::pos origin;
  if (mid_map == nullptr)
  {
    mid_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
    spl->add_map_cost(&problem,mid_map);

    //--- for line map init---
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_line_map = new EDTMap(origin,msg->width,edt_map_size);
    m_line_map->m_create_host_cpy = true;
    m_line_map->setup_device();

    // 3D rotation mode
    int m = m_line_map->m_map_size.x;
    int n = m_line_map->m_map_size.y;
    int p = m_line_map->m_map_size.z;
    int dim_0[3] = {m,n,p};
    int permu_0[3] = {1,0,2};
    int dim_1[3] = {n,m,p};
    int permu_1[3] = {0,2,1};
    int dim_2[3] = {n,p,m};
    int permu_2[3] = {2,0,1};
    cuttPlan(&m_rot_plan[0], 3, dim_0, permu_0, sizeof(int), nullptr);
    cuttPlan(&m_rot_plan[1], 3, dim_1, permu_1, sizeof(int), nullptr);
    cuttPlan(&m_rot_plan[2], 3, dim_2, permu_2, sizeof(int), nullptr);

    setup_map_msg(line_msg.line_map,m_line_map,true);
  }
  mid_map->copyEdtData(msg);
  m_line_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
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
CUDA_GEO::coord select_start_coord()
{
  // Default to the center of the map
  CUDA_GEO::coord start_crd(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);

  // Already recevied current ref_information
  if (received_ref)
  {
    CUDA_GEO::pos curr_pos(current_ref_state(0,0),
                           current_ref_state(0,1),
                           current_ref_state(0,2));
    CUDA_GEO::coord curr_crd = mid_map->pos2coord(curr_pos);

    // Check whether the current coord is inside the map
    if(mid_map->isInside(curr_crd))
      start_crd = curr_crd;
  }

  // If we are using the 2D mode, reset the coord's z value to the desired one
#ifdef USE2D
  int tgt_height_coord = mid_map->calcTgtHeightCoord(_2D_fly_height);
  start_crd.z = tgt_height_coord;
#endif

  return start_crd;
}
//---
CUDA_GEO::coord select_goal_coord(const CUDA_GEO::coord &start_crd)
{
  CUDA_GEO::coord goal_crd; // Target coord, for A* algorithm
  // Transform global goal to coord
  goal_crd = mid_map->pos2coord(goal);

  // Set the height of the coord to the desired flight height
  goal_crd.z = mid_map->calcTgtHeightCoord(_2D_fly_height);

  // Keep the target inside map
  goal_crd = mid_map->rayCast(start_crd,goal_crd, mid_map->getMaxX()*mid_map->getGridStep()*0.5*0.8).back();

  return goal_crd;
}
//---
Eigen::Matrix3d setup_ini_state(const std::vector<CUDA_GEO::pos> &guide_path)
{
  Eigen::Matrix3d s_ini;

  if (received_ref)
  {
    // If receved, use the current ref state as the initial state
    s_ini = current_ref_state;
  }
  else
  {
    // Otherwise, select the first point on the guide path and treat
    // the vehicle as static
    s_ini<<guide_path[0].x, guide_path[0].y, guide_path[0].z,
        0,0,0,
        0,0,0;
  }

  // If we are using the 2D mode, reset the z value, z velocity and z acc
#ifdef USE2D
  s_ini(0,2) = _2D_fly_height;
  s_ini(1,2) = 0;
  s_ini(2,2) = 0;
#endif

  return s_ini;
}
//---
Eigen::Matrix3d setup_ter_state(const std::vector<CUDA_GEO::pos> &guide_path)
{
  Eigen::Matrix3d s_ter;
  s_ter<<guide_path.back().x, guide_path.back().y, guide_path.back().z,
         0,0,0,
         0,0,0;
  return s_ter;
}
//---
void prepare_line_map(const std::vector<CUDA_GEO::pos> &trajectory)
{
  m_line_map->clearOccupancy();
  for (size_t i = 0; i < trajectory.size(); i++)
  {
    float3 pt = make_float3(trajectory[i].x,
                            trajectory[i].y,
                            trajectory[i].z);

    m_line_map->setOccupancy(pt,true);
  }
  CUDA_MEMCPY_H2D(m_line_map->m_sd_map,m_line_map->m_hst_sd_map,m_line_map->m_byte_size);
  GPU_EDT::edt_from_occupancy(m_line_map, m_rot_plan);
  CUDA_MEMCPY_D2H(m_line_map->m_hst_sd_map,m_line_map->m_sd_map,m_line_map->m_byte_size);
}
//---
void prepare_guidance_line_msg(const std::vector<CUDA_GEO::pos> &trajectory)
{
  // Clear the current line
  line_msg.pts.clear();

  // Put in the new line (as a vector of points)
  geometry_msgs::Point line_pnt;
  for (int i=0; i < trajectory.size(); i++)
  {
    line_pnt.x = trajectory[i].x;
    line_pnt.y = trajectory[i].y;
    line_pnt.z = trajectory[i].z;
    line_msg.pts.push_back(line_pnt);
  }

  // Prepare the line distance map
  prepare_line_map(trajectory);

  // Copy it to the mssage, the size is prelocated in the map callback function
  CUDA_MEMCPY_D2H(line_msg.line_map.payload8.data(),m_line_map->m_sd_map,m_line_map->m_byte_size);

  // Update the origin and grid step
  setup_map_msg(line_msg.line_map,m_line_map,false);

}
//---
void glb_plan(const ros::TimerEvent& msg)
{
  if (!received_cmd || !received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // Select the start and target coordinate
  CUDA_GEO::coord start = select_start_coord();
  CUDA_GEO::coord target = select_goal_coord(start);

  // Do the planning
  std::vector<CUDA_GEO::coord> path;
  planPath(start,target,path);
  std::reverse(path.begin(),path.end());

  //Get the guide path in pos instead of coord
  std::vector<CUDA_GEO::pos> guide_path; // The guide path directly produced by the A* search
  for (CUDA_GEO::coord &path_crd : path)
  {
    CUDA_GEO::pos path_pnt = mid_map->coord2pos(path_crd);
    guide_path.push_back(path_pnt);
  }

#ifdef SHOWPC
  publish_a_star_path(guide_path);
#endif

  if (guide_path.size() >= 2) //Do the trajectory smoothing as non-convex optimization
  {
    // Initialization
    Eigen::Matrix3d s_ini = setup_ini_state(guide_path);
    Eigen::Matrix3d s_ter = setup_ter_state(guide_path);
    spl->init_direct(s_ini,s_ter,1.0,guide_path,true);
    spl->update_start_cost();
    spl->update_finish_cost();

    //Solve
    ceres::Solve(options,&problem,&summary);

    //Get trajectory
    Eigen::Vector2d tr = spl->get_available_t_range();
    std::vector<double> t;
    for(double time = tr(0); time <= tr(1); time += 0.25)
      t.push_back(time);

    std::vector<CUDA_GEO::pos> trajectory = spl->get_trajectory_vec(t);

    // Debug info
    std::cout<<summary.BriefReport()<<std::endl;
    std::cout<<"-----Total time: "<<tr[1]<<std::endl;

    //Publish
    prepare_guidance_line_msg(trajectory);
    line_pub->publish(line_msg);
  }
  else
  {
    //Publish the guide line
    prepare_guidance_line_msg(guide_path);
    line_pub->publish(line_msg);
  }

#ifdef SHOWPC
  publish_line_map();
  publish_path(line_msg);
#endif

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Mid planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;
}
//---
void get_current_reference(const cpc_motion_planning::ref_data::ConstPtr &msg)
{
  received_ref = true;
  current_ref_state<<msg->data[0],msg->data[1],msg->data[2],
      msg->data[3],msg->data[4],msg->data[5],
      msg->data[6],msg->data[7],msg->data[8];
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  ros::NodeHandle nh;

  // Create the publisher
  line_pub = new ros::Publisher;
  line_vis_pub = new ros::Publisher;
  astar_vis_pub = new ros::Publisher;
  line_map_vis_pub = new ros::Publisher;
  *line_pub = nh.advertise<cpc_motion_planning::guide_line>("/mid_layer/guide_line",1);
  *line_vis_pub = nh.advertise<PointCloud>("/mid_layer/guide_line_vis",1);
  *astar_vis_pub = nh.advertise<PointCloud>("/mid_layer/astar_path_vis",1);
  *line_map_vis_pub = nh.advertise<PointCloud>("/mid_layer/line_map_vis",1);

  // Create the subscriber
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
  ros::Subscriber state_sub = nh.subscribe("/current_ref", 1, get_current_reference);

  // Other initialization
  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",_2D_fly_height,2.0);
  mid_safety_radius = 0.8f;

  //------------- Initialize the spline optimization-----------
  // Initialize the spline
  spl = new SPL_OPTI::Spline;
  spl->init(3,40,1.0,3,true);
  spl->add_vel_cost(&problem);
  spl->add_acc_cost(&problem);
  spl->add_jerk_cost(&problem);
  spl->add_start_cost(&problem);
  spl->add_finish_cost(&problem);
  spl->add_time_cost(&problem);

  //Set upper and lower boundary
  problem.SetParameterLowerBound(&(spl->m_beta),0,0.1);
  problem.SetParameterUpperBound(&(spl->m_beta),0,10.0);

  //Fix parameters
//  problem.SetParameterBlockConstant(&(spl->m_beta));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(0,0));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(0,1));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(0,2));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(spl->m_n,0));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(spl->m_n,1));
//  problem.SetParameterBlockConstant(&spl->m_ctrl_points(spl->m_n,2));
  //-------- Finish initializing the spline optimization-------

  // Create the mid planner timer
  glb_plan_timer = nh.createTimer(ros::Duration(1.0), &glb_plan);

  ros::spin();

  delete line_pub;
  delete line_vis_pub;
  delete line_map_vis_pub;
  delete astar_vis_pub;
  delete spl;
  if (mid_map)
  {
    delete mid_map;
  }
  if (m_line_map)
  {
    delete m_line_map;
    for (int i=0;i<3;i++)
      cuttDestroy(m_rot_plan[i]);
  }
  return 0;
}
