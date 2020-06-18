#include <loc_plan/uav_local_motion_planner.h>

UAVLocalMotionPlanner::UAVLocalMotionPlanner():
  m_stuck_pbty(0.0f),
  m_received_map(false),
  m_pose_received(false),
  m_edt_map(nullptr),
  m_fly_status(UAV::AT_GROUND)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UAVLocalMotionPlanner::map_call_back, this);
  m_pose_sub = m_nh.subscribe("/mavros/position/local", 1, &UAVLocalMotionPlanner::vehicle_pose_call_back, this);

}

UAVLocalMotionPlanner::~UAVLocalMotionPlanner()
{

}

void UAVLocalMotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
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

void UAVLocalMotionPlanner::vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}

void UAVLocalMotionPlanner::run_state()
{
  switch (m_fly_status) {
  case UAV::AT_GROUND:
  {
    do_at_ground();
    break;
  }
  case UAV::TAKING_OFF:
  {
    do_taking_off();
    break;
  }
  case UAV::IN_AIR:
  {
    do_in_air();
    break;
  }
  default:
    break;
  }
}

bool UAVLocalMotionPlanner::is_stuck(const std::vector<UAV::UAVModel::State> &traj, std::vector<JLT::State> yaw_traj, const float &best_cost)
{
    if (m_fly_status <= UAV::TAKING_OFF)
        return false;

    bool far_from_tgt = false;
    bool no_turning = false;
    bool no_moving_intention = false;

    // check is it far from target
    // TODO: make a better checking condition
    if (best_cost > 10)
        far_from_tgt = true;

    // check whether the vehicle is about to turn
    float max_turn = 0;
    float turn;
    JLT::State ini_yaw_state = yaw_traj[0];
    for (JLT::State yaw_state : yaw_traj)
    {
      turn = fabsf(yaw_state.p - ini_yaw_state.p);
      if (turn > max_turn)
          max_turn = turn;
    }
    if (max_turn < 0.25f)
        no_turning = true;

    // check whether the vehicle is about to move
    float max_dist = 0;
    float dist;
    UAV::UAVModel::State ini_s = traj[0];
    float3 shift = traj[0].p;
    for (UAV::UAVModel::State s : traj)
    {
        shift = ini_s.p - s.p;
        dist = sqrtf(dot(shift,shift));
        if (dist > max_dist)
            max_dist = dist;
    }
    if (max_dist < 0.4f)
        no_moving_intention = true;

    // update stuck probability
    if (far_from_tgt && no_turning && no_moving_intention)
        m_stuck_pbty +=0.15f;
    else
        m_stuck_pbty *=0.8f;

    if (m_stuck_pbty > 1)
    {
        m_stuck_pbty = 0;
        return true;
    }
    else
    {
        return false;
    }
}

