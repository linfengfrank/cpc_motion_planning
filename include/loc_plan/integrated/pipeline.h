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
public:
    Blackboard m_bb; // Blackboard for data communication
    int m_token = 0; // Token, only when token > 0, the task queue can run.
private:
    State * m_state; // The current state
    int m_cycle; // Execution cycle
    ros::Timer m_planning_timer; // Main planning timer
    ros::NodeHandle m_nh;

    // Helper functions
public:
    // Get the yaw angle
    float get_heading(const nav_msgs::Odometry &odom);


};

#endif // UGV_PLANNER_H
