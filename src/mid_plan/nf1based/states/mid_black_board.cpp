#include "mid_plan/nf1based/mid_black_board.h"

MidBlackboard::MidBlackboard():
  m_received_map(false),
  m_received_goal(false),
  m_drive_dir(-1),
  m_safety_radius(0.51f)
{
  m_nh.param<int>("/clp_idx_search_start",m_clp_idx_search_start,-50);
  m_nh.param<int>("/clp_idx_serach_end",m_clp_idx_search_end,50);
  m_nh.param<int>("/look_ahead",m_look_ahead,50);
  m_nh.param<int>("/look_back",m_look_back,-50);
  m_nh.param<float>("/curvature_split",m_curvature_split,0.2f);
  m_nh.param<float>("/mid_safety_radius",m_safety_radius,0.51f);
  m_nh.param<float>("/min_carrot_dist",m_min_carrot_dist,0.30f);
  m_nh.param<float>("/max_carrot_dist",m_max_carrot_dist,2.00f);

  m_map_sub = m_nh.subscribe("/edt_map", 1, &MidBlackboard::map_call_back,this);
  m_glb_tgt_sub = m_nh.subscribe("/set_global_goal", 1, &MidBlackboard::goal_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &MidBlackboard::slam_odo_call_back, this);
  m_glb_path_sub = m_nh.subscribe("/global_path",1, &MidBlackboard::glb_path_call_back, this);
  m_goal_reach_sub = m_nh.subscribe("/target_reached",1, &MidBlackboard::goal_reached_call_back, this);
  m_drive_dir_sub = m_nh.subscribe("/drive_dir",1,&MidBlackboard::drive_dir_call_back, this);
}

MidBlackboard::~MidBlackboard()
{

}

void MidBlackboard::setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize)
{
  msg.x_origin = map->getOrigin().x;
  msg.y_origin = map->getOrigin().y;
  msg.z_origin = map->getOrigin().z;
  msg.width = map->getGridStep();

  if (resize)
  {
    msg.x_size = map->getMaxX();
    msg.y_size = map->getMaxY();
    msg.z_size = 1;
    msg.payload8.resize(sizeof(CostTheta)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}


void MidBlackboard::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  m_received_map = true;
  CUDA_GEO::pos origin;
  if (m_d_map == nullptr)
  {
    m_d_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    setup_map_msg(m_nf1_msg.nf1,m_d_map,true);
    m_a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);

    //--- for line map init---
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_line_map = new EDTMap(origin,msg->width,edt_map_size);
    m_line_map->m_create_host_cpy = true;
    m_line_map->setup_device();

    int m = m_line_map->m_map_size.x;
    int n = m_line_map->m_map_size.y;
    int dim_0[2] = {m,n};
    int permu_0[2] = {1,0};
    int dim_1[2] = {n,m};
    int permu_1[2] = {1,0};
    cuttPlan(&m_rot_plan[0], 2, dim_0, permu_0, sizeof(int), nullptr);
    cuttPlan(&m_rot_plan[1], 2, dim_1, permu_1, sizeof(int), nullptr);
  }
  m_d_map->copyEdtData(msg);
  m_a_map->copyEdtData(msg);

  m_line_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
}

void MidBlackboard::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_received_goal= true;
  m_loiter_goal = CUDA_GEO::pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  float3 act_path_goal = make_float3(m_loiter_goal.x, m_loiter_goal.y, 0);
  update_nf1_guidance_map_info(cpc_aux_mapping::nf1_task::TYPE_FORWARD, cpc_aux_mapping::nf1_task::ID_LOITER,
                               m_loiter_goal_counter++, act_path_goal);
}

void MidBlackboard::glb_path_call_back(const cpc_motion_planning::path::ConstPtr &msg)
{
  if (m_received_path || m_path.request_ctt != msg->request_ctt)
  {
    std::cout<<"Global path: "<<msg->request_ctt<<" received."<<std::endl;
    // Read in the data files
    m_received_path = true;
    m_path = *msg;
  }
}

void MidBlackboard::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_curr_pose.x = msg->pose.pose.position.x;
  m_curr_pose.y = msg->pose.pose.position.y;

  double phi,theta,psi;
  tf::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  m_curr_pose.z = psi;
  if (!m_received_odom)
    m_mid_goal_orient = psi;

  m_received_odom = true;
}

void MidBlackboard::goal_reached_call_back(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  int path_id = msg->data[0];
  int act_id = msg->data[1];
//  if (path_id == m_path.request_ctt && act_id == m_curr_act_id
//      && m_curr_act_id+1 < m_path.actions.size())
//  {
//      m_curr_act_id++;
//      set_curr_act_path();
//  }
}

void MidBlackboard::drive_dir_call_back(const std_msgs::Int32::ConstPtr &msg)
{
  m_drive_dir = msg->data;
}
