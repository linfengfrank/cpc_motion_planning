#ifndef INTEGRATED_LOCAL_PLANNER_H
#define INTEGRATED_LOCAL_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit/pure_pursuit_ctrl.h>
#include <cpc_aux_mapping/nf1_task.h>

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <mpc/linear_mpc.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::NF1Evaluator,UGV::UGVSwarm<8>
namespace teb = teb_local_planner;
class IntLocalPlanner : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

public:
  IntLocalPlanner();
  ~IntLocalPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();
  virtual void do_pos_reached();
  virtual void do_fully_reached();
  virtual void do_dropoff();
private:
  void do_recover();
  void do_full_stuck();
  bool do_normal_teb();
  bool do_normal_pso();
private:
  void plan_call_back(const ros::TimerEvent&);
  void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg);
  void cycle_init();
  std::vector<double3> get_init_guess();
  bool get_smallest_child(const CUDA_GEO::coord &c, CUDA_GEO::coord &bc);

  bool check_tgt_is_same(const UGV::NF1Evaluator::Target &t1, const UGV::NF1Evaluator::Target &t2)
  {
    if (t1.act_id == t2.act_id && t1.path_id == t2.path_id)
      return true;
    else
      return false;
  }
  //---
  inline double pnt2line_dist(const double3 & c1, const double3 & c2, const double3 & c0)
  {
    double2 a = make_double2(c1.x-c0.x, c1.y-c0.y);
    double2 b = make_double2(c2.x-c1.x,c2.y-c1.y);

    double a_square = a.x*a.x + a.y*a.y;
    double b_square = b.x*b.x + b.y*b.y;
    double a_dot_b = a.x*b.x + a.y*b.y;

    if (b_square < 1e-3)
      return sqrt(a_square);

    return sqrt((a_square*b_square - a_dot_b*a_dot_b)/(b_square));
  }
  //---
  std::vector<size_t> split_merge(const std::vector<double3> &path, double th)
  {
    std::vector<size_t> split_idx;

    // If there is no path, return the center point
    if (path.size() <= 1)
    {
      return split_idx;
    }

    std::stack<std::pair<unsigned int,unsigned int>> task; // fist is anchor, second is target
    task.push(std::make_pair(path.size() - 1, 0));

    unsigned int check = static_cast<unsigned int>(path.size()) - 1;

    split_idx.push_back(path.size() - 1);
    while (task.size() > 0)
    {
      unsigned int target = task.top().second;
      unsigned int anchor = task.top().first;
      task.pop();
      // find the largest distance
      unsigned int max_id = 0;
      double max_dist = 0;
      for (unsigned int j = target; j< anchor; j++)
      {
        double dist = pnt2line_dist(path[anchor],path[target],path[j]);
        if (dist > max_dist)
        {
          max_dist = dist;
          max_id = j;
        }
      }

      if (max_dist > th)
      {
        task.push(std::make_pair(max_id, target));
        task.push(std::make_pair(anchor, max_id));
        target = max_id;
      }
      else
      {
        split_idx.push_back(target);
        if (target >= check)
        {
          std::cout<<"path size: "<<path.size()<<std::endl;
          ROS_ERROR("Wrong split sequence");
          exit(-1);
        }
        check = target;
      }
    }
    std::reverse(split_idx.begin(),split_idx.end());
    return split_idx;
  }

  //---
  float acc_filter(float curr_v, float tgt_v, const float acc_lim, const float dt)
  {
    float achieved_v;
    float desired_acc = (tgt_v - curr_v)/dt;

    if(fabsf(desired_acc) < acc_lim)
    {
      achieved_v = tgt_v;
    }
    else
    {
      if (desired_acc > 0)
        achieved_v = curr_v + acc_lim*dt;
      else
        achieved_v = curr_v - acc_lim*dt;
    }
    return achieved_v;
  }

  //---
  bool smooth_reference(const UGV::UGVModel::State &ini_state, const std::vector<teb::Reference> &raw_ref,
                        std::vector<UGV::UGVModel::State> &final_ref, bool use_simple_filter);
private:
  ros::Subscriber m_nf1_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_stuck_plan_request_pub;
  ros::Publisher m_drive_dir_pub;

  bool m_goal_received;
  bool m_task_is_new;
  bool m_use_de;
  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
  UGV::NF1Evaluator::Target m_goal;
  UGV::UGVModel::State m_carrot;
  float m_ref_v, m_ref_w, m_ref_theta;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt, m_tht_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  int m_stuck_start_cycle;
  int m_full_start_cycle;
  int m_plan_request_cycle;
  int m_swarm_size;
  int m_batch_num;
  int m_episode_num;
  NF1MapDT *m_nf1_map;
  cpc_motion_planning::path m_stuck_recover_path;
  UGVRecMotionPlanner m_recover_planner;
  ros::ServiceClient m_collision_check_client;
  STUCK_SUB_MODE m_stuck_submode;
  teb::HomotopyClassPlannerPtr m_teb_planner;
  teb::TebConfig m_cfg;
  teb::TebVisualizationPtr m_visualization;

  int N_hor;
  linear_mpc* m_mpc;
  bool m_use_simple_filter;
};

#endif // INTEGRATED_LOCAL_PLANNER_H
