#ifndef BRAKE_STATE_H
#define BRAKE_STATE_H

#include <loc_plan/integrated/state.h>
#include <iostream>
#include <cpc_motion_planning/pso/pso_planner.h>

class LocalPlannerPipeline;
// Every state has to be a singleton class
class BrakeState : public State
{
    // Propositoin enum
    // must start with 0 and end with the MAX_SIZE
    // cannot assign custom values
    enum PPS
    {
        TIME_UP = 0,
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
    BrakeState();
    BrakeState(const BrakeState& other);
    BrakeState& operator=(const BrakeState& other);

private:
    LocalPlannerPipeline *m_p = nullptr;
    std::vector<UGV::UGVModel::State> m_stuck_traj;
private:
    int m_brake_start_cycle;
    // Proposition evaluation functions
    bool check_time_up();


};


#endif // BRAKE_STATE_H
