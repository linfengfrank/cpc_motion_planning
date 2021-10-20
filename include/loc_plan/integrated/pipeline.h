#ifndef UGV_PLANNER_H
#define UGV_PLANNER_H
#include <loc_plan/integrated/state.h>
#include <loc_plan/integrated/black_board.h>
#include <ros/ros.h>


class Pipeline
{
public:
    Pipeline();

    // The entrance function that run our state machine
    void execute(const ros::TimerEvent&);

    // Preparation function called per cycle
    void prepare_cycle();

    // Finishing function called per cycle
    void finish_cycle();

    // Add a token so that the task queue can run again
    void add_token()
    {
        m_token++;
    }

    // Get the current execute cycle
    int get_cycle()
    {
        return m_cycle;
    }

private:
    State * m_state; // The current state
    int m_cycle; // Execution cycle
    ros::Time m_cycle_start_time;
    ros::Timer m_planning_timer; // Main planning timer
    ros::NodeHandle m_nh;

public:
    Blackboard m_bb; // Blackboard for data communication
    int m_token = 0; // Token, only when token > 0, the task queue can run.
    // Helper variables
    float m_stuck_pbty = 0;
    float m_lowpass_stuck_pbty = 0;
    float m_ref_v = 0;
    float m_ref_w = 0;
    float m_ref_theta = 0;
    int m_ref_start_idx = 0; // The starting index of this cycle's reference
    int m_ref_gen_count = 0; // Number of times of generating reference

    // Helper functions
public:
    //
    void increase_ref_gen_count()
    {
      m_ref_start_idx = get_next_ref_start_idx();
      m_ref_gen_count++;
    }

    // Get the reference start idx of the next planning cycle
    int get_next_ref_start_idx()
    {
      return (m_ref_gen_count+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
    }

    // Get the yaw angle
    float get_heading(const nav_msgs::Odometry &odom);

    // Prepare then return the initial state of the vehicle
    UGV::UGVModel::State prepare_init_state();

    // Predicte the state given current measurment in Odometry(odom) format
    UGV::UGVModel::State predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx, bool is_heading_ref);

    // Select wether to use measurement or reference to be assigned to the current state
    float select_mes_ref(float mes, float ref, float th = 1.0f, int ctt_th = 5);

    // Select wether to use measurement or reference to be assigned to the current state (yaw)
    float select_mes_ref_heading(bool &is_heading_ref, float mes, float ref, float th = 1.0f, int ctt_th = 5);

    float in_pi(float in)
    {
      return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
    }

    // Signum function
    template <typename T> int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    // Generate a full stop trajectory, stop at the current state (curr_s)
    void full_stop_trajectory(std::vector<UGV::UGVModel::State> &traj, UGV::UGVModel::State curr_s);

    // This function check whether there is a collision by simulating a tracking of m_traj from the
    // true initial state (aka. consider the tracking error).
    bool is_tracking_safe(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref);

    float tracking_min_edt(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref,
                                                          float yaw_ctrl_gain, float w_scale, float exam_time = 4.0f);

    std::vector<UGV::UGVModel::State> simulate_tracking(const std::vector<UGV::UGVModel::State> &ref, const nav_msgs::Odometry &odom,
                                                        float yaw_ctrl_gain, float w_scale, float exam_time);

    float traj_min_edt(const std::vector<UGV::UGVModel::State> &traj);

    float get_dist_from_host_edt(const UGV::UGVModel::State &s) const;

    void calculate_bounding_centres(const UGV::UGVModel::State &s, float2 &c_r, float2 &c_f) const;

    bool is_stuck(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);

    bool is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);

    bool is_stuck_lowpass(const UGV::UGVModel::State& s, const UGV::UGVModel::State &tgt_state);

    bool is_pos_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_radius = 0.8f);

    void update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);

    std::vector<StampedUGVState> make_stamped_reference(int start_idx, const std::vector<UGV::UGVModel::State> &raw_ref);

    void update_norminal_states(const std::vector<StampedUGVState> &stamped_ref);

    int bool_to_drive_type(bool is_forward);
};

#endif // UGV_PLANNER_H
