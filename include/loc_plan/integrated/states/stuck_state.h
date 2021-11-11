#ifndef STUCK_STATE_H
#define STUCK_STATE_H

#include <cpc_motion_planning/state.h>
#include <iostream>
#include <cpc_motion_planning/pso/pso_planner.h>

class LocalPlannerPipeline;
class NormalPsoState;
// Every state has to be a singleton class
class StuckState : public State
{
    // Propositoin enum
    // must start with 0 and end with the MAX_SIZE
    // cannot assign custom values
    enum PPS
    {
        TIME_UP = 0,
        SUCCESS,
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
    StuckState();
    StuckState(const StuckState& other);
    StuckState& operator=(const StuckState& other);

private:
    LocalPlannerPipeline *m_p = nullptr;
    NormalPsoState *m_pso = nullptr;
    std::vector<UGV::UGVModel::State> m_stuck_traj;
private:
    int m_stuck_start_cycle;
    bool m_plan_success;
    // Proposition evaluation functions
    bool check_time_up();
    bool check_success();

};


#endif // STUCK_STATE_H
