#ifndef REACH_STATE_H
#define REACH_STATE_H

#include <cpc_motion_planning/state.h>
#include <iostream>
#include <cpc_motion_planning/pso/pso_planner.h>

class LocalPlannerPipeline;
// Every state has to be a singleton class
class ReachState : public State
{
    // Propositoin enum
    // must start with 0 and end with the MAX_SIZE
    // cannot assign custom values
    enum PPS
    {
        NEW_TARGET = 0,
        MAX_SIZE
    };
public:
    void on_execute() override;
    void on_finish() override;
    State& toggle() override;
    static State & getInstance(); // Get instance of the singleton class
    void attach_to_pipe(Pipeline *p) override;

    void on_activation() override;
    void on_deactivation() override {}

private:
    // private constructor, copy constructor and = operator for the singleton class
    ReachState();
    ReachState(const ReachState& other);
    ReachState& operator=(const ReachState& other);

private:
    LocalPlannerPipeline *m_p = nullptr;
    std::vector<UGV::UGVModel::State> m_reach_traj;
private:
    // Proposition evaluation functions
    bool check_new_target();


};

#endif // REACH_STATE_H
