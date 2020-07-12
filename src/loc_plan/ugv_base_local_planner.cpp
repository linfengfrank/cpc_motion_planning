#include <loc_plan/ugv_base_local_planner.h>

UGVLocalMotionPlanner::UGVLocalMotionPlanner():
  m_received_map(false),
  m_raw_odo_received(false),
  m_slam_odo_received(false),
    m_edt_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UGVLocalMotionPlanner::map_call_back, this);
  m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &UGVLocalMotionPlanner::raw_odo_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &UGVLocalMotionPlanner::slam_odo_call_back, this);
  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";
}

UGVLocalMotionPlanner::~UGVLocalMotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }
}

void UGVLocalMotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_received_map = true;
  if (m_edt_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->setup_device();
  }
  else
  {
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
  }
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
}

void UGVLocalMotionPlanner::raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_raw_odo_received = true;
  m_raw_odo = *msg;
}

void UGVLocalMotionPlanner::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_slam_odo_received = true;
  m_slam_odo = *msg;
}
