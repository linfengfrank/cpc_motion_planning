#include <loc_plan/integrated/black_board.h>
#include <std_msgs/Int32.h>

Blackboard::Blackboard()
{
  // Read in the parameters
  m_nh.param<double>("/turning_efficiency",m_turning_efficiency,1.0);
  m_nh.param<bool>("/use_adrc",m_use_adrc,true);
  m_nh.param<float>("cpc_footprint_offset",m_footprint_offset,0.25f);

  // Add in the subscribers
  m_nf1_sub = m_nh.subscribe("/nf1",1,&Blackboard::nf1_call_back, this);
  m_map_sub = m_nh.subscribe("/edt_map", 1, &Blackboard::map_call_back, this);
  m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &Blackboard::raw_odo_call_back, this);
#ifdef ADD_DELAY
  m_sub.subscribe(m_nh, "/slam_odom", 1);
  m_seq = new message_filters::TimeSequencer<nav_msgs::Odometry> (m_sub, ros::Duration(0.2), ros::Duration(0.01), 10);
  m_seq->registerCallback(&Blackboard::slam_odo_call_back, this);
#else
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &Blackboard::slam_odo_call_back, this);
#endif

  // Add in the publishers
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_simulate_traj_pub = m_nh.advertise<PointCloud> ("simulated_traj", 1);
  m_drive_dir_pub = m_nh.advertise<std_msgs::Int32>("drive_dir",1);

  // Initialize point cloud to show the trajectory
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";

  // Other initialization
  m_ref_msg.rows = 5;
}

void Blackboard::publish_status_info(std::string &str)
{
  //Publish the ref status string for loging
  std_msgs::String msg;
  msg.data = str;
  m_status_pub.publish(msg);
}

void Blackboard::plot_ref_trajectory(const std::vector<UGV::UGVModel::State> &traj)
{
  for (UGV::UGVModel::State traj_s : traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = 0;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
}

void Blackboard::plot_sim_trajectory(const std::vector<UGV::UGVModel::State> &traj)
{
  for (UGV::UGVModel::State traj_s : traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = 0;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_simulate_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
}

void Blackboard::publish_reference(const std::vector<StampedUGVState>& stamped_traj)
{
  // Clear old msg
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();
  m_ref_msg.time.clear();

  int cols = 0;
  for (StampedUGVState traj_s : stamped_traj)
  {
    add_to_ref_msg(m_ref_msg,traj_s.id,traj_s.s,traj_s.t);
    cols++;
  }
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);

  // Also publish the executed driving direction
  std_msgs::Int32 exec_drive_dir;
  exec_drive_dir.data = m_exec_drive_dir;
  m_drive_dir_pub.publish(exec_drive_dir);
}

void Blackboard::nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg)
{
  m_goal_received = true;
  // Setup the NF1 map
  if (m_nf1_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    int3 m_nf1_map_size = make_int3(msg->nf1.x_size,msg->nf1.y_size,msg->nf1.z_size);
    m_nf1_map = new NF1MapDT(origin,msg->nf1.width,m_nf1_map_size);
    m_nf1_map->m_create_host_cpy = true;
    m_nf1_map->setup_device();
  }
  else
  {
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    m_nf1_map->m_grid_step = msg->nf1.width;
  }
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));
  memcpy(m_nf1_map->m_hst_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));

  // Setup the drive type
  m_task_drive_dir = msg->drive_type;

  // Setup the goal
  if(m_goal.act_id != msg->act_id || m_goal.path_id != msg->path_id)
  {
    m_task_is_new = true;
    m_goal.s.p.x = msg->goal_x;
    m_goal.s.p.y = msg->goal_y;
    m_goal.s.theta = msg->goal_theta;
    m_goal.path_id = msg->path_id;
    m_goal.act_id = msg->act_id;
    m_goal.reaching_radius = 0.5f;
  }

  // Setup the carrot
  m_carrot.p.x = msg->carrot_x;
  m_carrot.p.y = msg->carrot_y;
  m_carrot.theta = msg->carrot_theta;
}

void Blackboard::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_map_received = true;
  if (m_edt_map == nullptr)
  {
    // Construct and initialize the edt map
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->m_create_host_cpy = m_create_host_edt;
    m_edt_map->setup_device();
  }
  else
  {
    // Set the edt map's origin location
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
  }

  // Copy map data to the device
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));

  // Copy map data to the host
  if (m_create_host_edt)
    memcpy(m_edt_map->m_hst_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
}

void Blackboard::raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_raw_odo_received = true;
  m_raw_odo = *msg;
}

void Blackboard::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_slam_odo_received = true;
  m_slam_odo = *msg;
}


void Blackboard::add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj_s, const ros::Time &t)
{
  ref_msg.ids.push_back(ref_counter);
  ref_msg.time.push_back(t.toSec());
  ref_msg.data.push_back(traj_s.v);
  ref_msg.data.push_back(traj_s.w);
  ref_msg.data.push_back(traj_s.theta);
  ref_msg.data.push_back(traj_s.p.x);
  ref_msg.data.push_back(traj_s.p.y);
}
