#ifndef UAV_NF1_MOTION_PLANNER_H
#define UAV_NF1_MOTION_PLANNER_H

#include <loc_plan/uav_local_motion_planner.h>
#define SIMPLE_UAV_NF1 UAV::UAVModel,UAV::UAVDPControl,UAV::NF1Evaluator,UAV::UAVSwarm<1>
#define EMERGENT_UAV_NF1 UAV::UAVModel,UAV::UAVJLTControl,UAV::NF1Evaluator,UAV::UAVSwarm<1>

class UAVNF1MotionPlanner : public UAVLocalMotionPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  UAVNF1MotionPlanner();
  ~UAVNF1MotionPlanner();

protected:
  virtual void do_at_ground();
  virtual void do_taking_off();
  virtual void do_in_air();

private:
  void plan_call_back(const ros::TimerEvent&);
  void goal_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void calculate_yaw(const std::vector<UAV::UAVModel::State> &traj);

private:


  ros::Subscriber m_goal_sub;
  ros::Timer m_planning_timer;


  ros::Publisher m_traj_pub;
  ros::Publisher m_ctrl_pub;
  ros::Publisher m_ref_pub;
  ros::Publisher topology_paths_pub;


  bool m_goal_received;

  NF1Map *m_nf1_map;

  PSO::Planner<SIMPLE_UAV_NF1> *m_pso_planner;
  PSO::Planner<EMERGENT_UAV_NF1> *m_emergent_planner;


  PointCloud::Ptr m_traj_pnt_cld, m_ctrl_pnt_cld;
  //UAV::SingleTargetEvaluator::Target m_goal;
  UAV::UAVModel::State m_curr_ref;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_plan_cycle;
  int m_ref_start_idx;
  UAVHeadingSolver m_head_sov;


};

#endif // UAV_NF1_MOTION_PLANNER_H
