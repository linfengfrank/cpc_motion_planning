#include "loc_plan/uav_nf1_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

UAVNF1MotionPlanner::UAVNF1MotionPlanner():
  UAVLocalMotionPlanner(),
  m_goal_received(false),
  m_nf1_map(nullptr),
  m_planning_started(false)
{
  // Read parameters
  m_nh.param<float>("/nndp_cpp/fly_height",m_take_off_height,2.0);
  m_nh.param<float>("/nndp_cpp/leap_height",m_leap_height,0.4);

  // Advertise subscribe
  m_nf1_sub = m_nh.subscribe("/mid_layer/goal",1,&UAVNF1MotionPlanner::nf1_call_back, this);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVNF1MotionPlanner::plan_call_back, this);

  // Initialize the planners
  m_pso_planner = new PSO::Planner<SIMPLE_UAV_NF1>(150,30,1);
  m_pso_planner->initialize();

  m_emergent_planner = new PSO::Planner<EMERGENT_UAV_NF1>(50,20,1);
  m_emergent_planner->initialize();
  m_emergent_planner->m_ctrl_dev.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));
  m_emergent_planner->m_ctrl_host.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));

  //Initialize the control message
  m_ref_msg.rows = 12;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UAVNF1MotionPlanner::~UAVNF1MotionPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;

  m_emergent_planner->release();
  delete m_emergent_planner;
}

void UAVNF1MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  cycle_init();
  cycle_process_based_on_status();
  if (!m_planning_started)
    return;

  if (m_fly_status <= UAV::AT_GROUND)
    return;

  // trajectory generation
  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
  float t = 0.0f;
  int i=0;
  for (UAV::UAVModel::State traj_s : m_traj)
  {
    t += PSO::PSO_CTRL_DT;
    JLT::State yaw_state = m_yaw_traj[i++];

    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s,yaw_state);

    if (ref_counter == next_ref_start_idx)
    {
      m_curr_ref = traj_s;
      m_curr_yaw_ref = yaw_state;
    }
    cols++;
  }

  m_ref_start_idx = next_ref_start_idx;
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_plan_cycle++;
#ifdef SHOWPC
  plot_trajectory(m_traj);
#endif
}

void UAVNF1MotionPlanner::nf1_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  // During taking off the carrot position shall not be changed (set at the at_ground state)
  // so directly return at this stage.
  if (m_fly_status == UAV::TAKING_OFF)
    return;

  m_goal_received = true;
  if (m_nf1_map == nullptr)
  {
    // Initialize the NF1 map for PSO planner
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 m_nf1_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_nf1_map = new NF1Map(origin,msg->width,m_nf1_map_size);
    m_nf1_map->setup_device();
  }
  else
  {
    // Set the NF1 map's origin and grid step
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_nf1_map->m_grid_step = msg->width;
  }

  // Copy the actual map data to GPU device
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));

  // Set the map pointer for the evaluators
  m_pso_planner->m_eva.m_nf1_map = *m_nf1_map;
  m_emergent_planner->m_eva.m_nf1_map = *m_nf1_map;

  // Set the carrot for the evaluators (they will be used to set the desired fly height
  // in the optimization process)
  float3 nf1_carrot = make_float3(msg->x_carrot, msg->y_carrot, msg->z_carrot);
  m_pso_planner->m_eva.m_carrot = nf1_carrot;
  m_emergent_planner->m_eva.m_carrot = nf1_carrot;
}

void UAVNF1MotionPlanner::do_at_ground()
{
  if (m_pose_received && m_received_map && m_goal_received)
  {
    //set the current state from pose
    convert_init_pose(m_pose,m_curr_ref,m_curr_yaw_ref);
    //to make the height refernece "leap" a little bit
    //during the takeoff
    m_curr_ref.p.z += m_leap_height;
    set_init_state(m_curr_ref, m_curr_yaw_ref);

    //set the yaw target as the current target
    m_head_sov.set_yaw_target(m_curr_yaw_ref.p);

    //create the take_off carrot
    float3 take_off_carrot = m_curr_ref.p;
    take_off_carrot.z = m_take_off_height;

    //assign the take_off_carrot to the planners
    m_pso_planner->m_eva.m_carrot = take_off_carrot;
    m_emergent_planner->m_eva.m_carrot = take_off_carrot;

    // Egage and takeoff
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
    m_fly_status = UAV::TAKING_OFF;
  }
}
void UAVNF1MotionPlanner::do_taking_off()
{
  auto start = std::chrono::steady_clock::now();

  // The reference can now be genereated and published
  m_planning_started = true;

  // During taking off the carrot position shall not be changed (set at the at_ground state)
  // in goal call back, it returns directly in this stage.
  calculate_trajectory<SIMPLE_UAV_NF1>(m_pso_planner,m_traj);

  // The target heading shall not be changed as well (set at the at_ground state)
  m_yaw_traj = m_head_sov.generate_yaw_traj();

  // Condition for finishing takeoff
  if (m_curr_ref.p.z >= m_take_off_height - 0.2f && fabsf(m_curr_ref.v.z)<0.3f)
  {
    // In the NF1 planner, since we assume we are using a 2D Lidar (for now)
    // we only turn on obstacle avoidance but not the field of view constraints
    m_pso_planner->m_eva.m_in_air = true;
    m_pso_planner->m_eva.m_consider_fov = false;

    m_emergent_planner->m_eva.m_in_air = true;
    m_emergent_planner->m_eva.m_consider_fov = false;
    m_fly_status = UAV::IN_AIR;
  }

  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_in_air()
{
  auto start = std::chrono::steady_clock::now();
  // Plan the trajectory
  calculate_trajectory<SIMPLE_UAV_NF1>(m_pso_planner,m_traj);

  // If the generated trajectory is unsafe
  if (m_pso_planner->result.collision)
  {
    // Go to the emergency mode and use the emergency planner
    m_fly_status = UAV::EMERGENT;
    cycle_process_based_on_status();
  }
  else
  {
    // Since we assume we are using a 2D lidar, we fix the yaw during the
    // entire flight.
    //m_head_sov.cal_yaw_target(m_pso_planner->result.best_loc[0]);
    m_yaw_traj = m_head_sov.generate_yaw_traj(); // Generate the yaw trajectory here
    if(is_stuck(m_traj, m_yaw_traj, m_pso_planner->result.best_cost))
    {
      // Stuck handling
      m_fly_status = UAV::STUCK;
      m_start_stuck_cycle = m_plan_cycle;
    }
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (in air): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_emergent()
{
  auto start = std::chrono::steady_clock::now();
  // Emergent planner will try find a trajectory with increased acceleration
  // and jerk limits.
  calculate_trajectory<EMERGENT_UAV_NF1>(m_emergent_planner,m_traj);
  if(m_emergent_planner->result.collision)
  {
    // If the emergency planner still fails go to the braking mode
    m_fly_status = UAV::BRAKING;
    m_start_braking_cycle = m_plan_cycle;

    // Braking point is determined by the current vehicle pose
    m_curr_ref = odom2state(m_pose);
    cycle_process_based_on_status();
  }
  else
  {
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (emergent): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_braking()
{
  auto start = std::chrono::steady_clock::now();
//  m_rep_filed.generate_repulse_traj(m_traj, *m_edt_map, m_curr_ref);

  // Just stop and brake
  generate_static_traj(m_traj, m_curr_ref);

  // After a while go to stuck mode
  if (m_plan_cycle - m_start_braking_cycle > 20)
  {
    // Stuck handling
    m_fly_status = UAV::STUCK;
    m_start_stuck_cycle = m_plan_cycle;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (braking): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;

}

void UAVNF1MotionPlanner::do_stuck()
{
  std::cout<<"stuck"<<std::endl;

  m_pso_planner->m_eva.m_stuck = true;
  calculate_trajectory<SIMPLE_UAV_NF1>(m_pso_planner, m_traj);

  //Goto: Braking
  if (m_pso_planner->result.collision)
  {
    // If the stuck planner fails go to the braking mode
    m_fly_status = UAV::BRAKING;
    m_pso_planner->m_eva.m_stuck = false;
    m_start_braking_cycle = m_plan_cycle;

    // Braking point is determined by the current vehicle pose
    m_curr_ref = odom2state(m_pose);
    cycle_process_based_on_status();
  }

  //Goto: Normal
  if (m_plan_cycle - m_start_stuck_cycle >= 10)
  {
    m_fly_status = UAV::IN_AIR;
    m_pso_planner->m_eva.m_stuck = false;
  }
}

void UAVNF1MotionPlanner::set_init_state(const UAV::UAVModel::State& trans, const JLT::State &yaw)
{
  // Set the initial states for all planners
  m_pso_planner->m_model.set_ini_state(trans);
  m_pso_planner->m_eva.m_curr_yaw = yaw.p;
  m_pso_planner->m_eva.m_curr_pos = trans.p;

  m_emergent_planner->m_model.set_ini_state(trans);
  m_emergent_planner->m_eva.m_curr_yaw = yaw.p;
  m_emergent_planner->m_eva.m_curr_pos = trans.p;

  m_head_sov.set_yaw_state(yaw);
}

void UAVNF1MotionPlanner::cycle_init()
{
  // Set the initial translation and yaw state
  set_init_state(m_curr_ref, m_curr_yaw_ref);

  // Construct default trajectories for pos and yaw
  generate_static_traj(m_traj, m_curr_ref);
  m_yaw_traj = m_head_sov.generate_yaw_traj();
}



