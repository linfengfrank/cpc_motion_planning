#ifndef START_IDLE_STATE_H
#define START_IDLE_STATE_H

#include <loc_plan/integrated/state.h>
#include <iostream>

class LocalPlannerPipeline;

// Every state has to be a singleton class
class StartIdleState : public State
{
    // Propositoin enum
    // must start with 0 and end with the MAX_SIZE
    // cannot assign custom values
    enum PPS
    {
        READY_TO_GO = 0,
        MAX_SIZE
    };
public:
    void on_execute() override;
    void on_finish() override;
    State& toggle() override;
    static State & getInstance(); // Get instance of the singleton class
    void attach_to_pipe(Pipeline *p) override;

    void on_activation() override {}
    void on_deactivation() override {}

private:
    // private constructor, copy constructor and = operator for the singleton class
    StartIdleState();
    StartIdleState(const StartIdleState& other);
    StartIdleState& operator=(const StartIdleState& other);

private:
    LocalPlannerPipeline *m_p = nullptr;

private:
    // Proposition evaluation functions
    bool check_ready_to_go();

};

#endif // START_IDLE_STATE_H
