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

UGV::UGVModel::State UGVLocalMotionPlanner::predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx)
{
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;
#ifdef PRED_STATE
  // Find the most related cmd
  while (m_cmd_q.size()>0)
  {
    if (m_cmd_q.front().t.toSec() <= odom.header.stamp.toSec())
      m_cmd_q.pop_front();
    else
      break;
  }

  for (const CmdLog &tmp : m_cmd_q)
  {
    if (tmp.id < ref_start_idx)
    {
      s.p.x = s.p.x + tmp.v*cos(s.theta)*PSO::PSO_CTRL_DT;
      s.p.y = s.p.y + tmp.v*sin(s.theta)*PSO::PSO_CTRL_DT;
      s.theta = s.theta + tmp.w*PSO::PSO_CTRL_DT;
    }
    else
    {
      break;
    }
  }
#endif
  return s;
}

void UGVLocalMotionPlanner::update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
#ifdef PRED_STATE
  if(m_cmd_q.empty())
  {
    load_into_queue(ref, curr_t);
  }
  else
  {
    int diff = ref.ids[0] - m_cmd_q.front().id;
    while(static_cast<int>(m_cmd_q.size()) > diff && !m_cmd_q.empty())
    {
      m_cmd_q.pop_back();
    }
    load_into_queue(ref, curr_t);
  }
#endif
}

#ifdef PRED_STATE
void UGVLocalMotionPlanner::load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
  for (int i=0; i<ref.cols; i++)
  {
    CmdLog tmp;
    tmp.t = curr_t + ros::Duration((i+1)*PSO::PSO_CTRL_DT);

    tmp.id = ref.ids[i];
    tmp.v = ref.data[i*ref.rows];
    tmp.w = ref.data[i*ref.rows + 1];
    //std::cout<<"id: "<<ref.ids[i]<<", "<<tmp.t<<std::endl;
    m_cmd_q.push_back(tmp);
  }
}
#endif

