#ifndef PSO_MPC_H
#define PSO_MPC_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit/pure_pursuit_ctrl.h>
#include <cpc_aux_mapping/nf1_task.h>
#include <deque>
#define PSO_MPC_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::TFEvaluator,UGV::UGVSwarm<8>
class PsoMpc : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

  struct ref_state
  {
    int idx;
    float time;
    std::vector<float> data;
  };

public:
  PsoMpc();
  ~PsoMpc();

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
private:
  void plan_call_back(const ros::TimerEvent&);
  void ref_traj_callback(const cpc_motion_planning::ref_data::ConstPtr &msg);
  void cycle_init();
  bool check_tgt_is_same(const UGV::NF1Evaluator::Target &t1, const UGV::NF1Evaluator::Target &t2)
  {
    if (t1.act_id == t2.act_id && t1.path_id == t2.path_id)
      return true;
    else
      return false;
  }

  void load_into_queue(std::deque<ref_state> &queue, const cpc_motion_planning::ref_data &ref)
  {
    for (int i=0; i<ref.cols; i++)
    {
      std::vector<float> col;
      for (int j=0; j<ref.rows; j++)
      {
        col.push_back(ref.data[i*ref.rows+j]);
      }
      ref_state tmp;
      tmp.idx = ref.ids[i];
      tmp.data = col;
      tmp.time = ref.time[i];
      queue.push_back(tmp);
    }
  }

  void set_ref_to_queue(const cpc_motion_planning::ref_data::ConstPtr &msg, std::deque<ref_state> &queue)
  {
    if(queue.empty())
    {
      load_into_queue(queue, *msg);
    }
    else
    {
      int diff = msg->ids[0] - queue.front().idx;
      while(queue.size() > diff && !queue.empty())
      {
        queue.pop_back();
      }
      load_into_queue(queue, *msg);
    }
  }

private:
  ros::Subscriber m_ref_traj_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_stuck_plan_request_pub;
  ros::Publisher m_force_reset_pub;

  bool m_goal_received;
  bool m_task_is_new;
  bool m_use_de;
  PSO::Planner<PSO_MPC_UGV> *m_pso_planner;
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
  std::deque<ref_state> m_ref_queue;
};

#endif // PSO_MPC_H
