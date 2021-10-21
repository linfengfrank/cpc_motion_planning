#ifndef UGV_PLANNER_H
#define UGV_PLANNER_H
#include <loc_plan/integrated/state.h>
#include <ros/ros.h>


class Pipeline
{
public:
    Pipeline();

    virtual ~Pipeline();

    // The entrance function that run our state machine
    virtual void execute(const ros::TimerEvent&);

    // Preparation function called per cycle
    virtual void prepare_cycle() = 0;

    // Finishing function called per cycle
    virtual void finish_cycle() = 0;

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

    void start_timer()
    {
      //Start the timer
      m_planning_timer = m_nh.createTimer(ros::Duration(0.2), &Pipeline::execute, this);
    }

protected:
    State * m_state; // The current state
    int m_cycle; // Execution cycle
    ros::Time m_cycle_start_time;
    ros::Timer m_planning_timer; // Main planning timer
    ros::NodeHandle m_nh;

public:

    int m_token = 0; // Token, only when token > 0, the task queue can run.
};

#endif // UGV_PLANNER_H
