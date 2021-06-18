#include "pure_pursuit/pure_pursuit_ctrl.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>

UGVRecMotionPlanner::UGVRecMotionPlanner()
{
  m_pso_planner = new PSO::Planner<RECOVER_UGV>(120,50,3);

  m_curr_action_id = 0;

  ros::NodeHandle nh;
  m_carrot_pub = nh.advertise<geometry_msgs::PoseStamped>("/carrot",1);
}

void UGVRecMotionPlanner::init_swarm(int step_num, float step_dt, float var_s, float var_theta, std::string file_location)
{
  // Init swarm
  m_pso_planner->m_swarm.set_step_dt(step_num,step_dt);
  m_pso_planner->m_swarm.set_var(make_float3(var_s,var_theta,1.0f));
  m_pso_planner->m_file_location = file_location;
  m_pso_planner->initialize();
}

UGVRecMotionPlanner::~UGVRecMotionPlanner()
{
  delete m_pso_planner;
}

bool UGVRecMotionPlanner::calculate_trajectory(const UGV::UGVModel::State &curr_s, UGV::UGVModel::State goal,
                                               UGV::UGVModel::State carrot, EDTMap *edt_map, std::vector<UGV::UGVModel::State> &traj)
{
  //Update initial state
  m_pso_planner->m_model.set_ini_state(curr_s);

  float2 diff = make_float2(carrot.p.x - curr_s.p.x, carrot.p.y - curr_s.p.y);
  float diff_dist = sqrtf(dot(diff,diff));

  if (diff_dist > 0.25f)
    carrot.theta = atan2f(diff.y,diff.x);
  else
    carrot.theta = curr_s.theta;

  //Set the carrot
  m_pso_planner->m_eva.setGoal(goal);
  m_pso_planner->m_eva.setCarrot(carrot);
  m_pso_planner->m_eva.m_target_received = true;

  //Do the planning
  m_pso_planner->plan(*edt_map);

  //Trajectory generation
  traj = m_pso_planner->generate_trajectory();

  return true;
}
