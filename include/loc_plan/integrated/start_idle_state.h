#ifndef START_IDLE_STATE_H
#define START_IDLE_STATE_H

#include <loc_plan/integrated/state.h>
#include <iostream>

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
    void on_enter(Pipeline* pipe);
    void on_exit(Pipeline* pipe);
    State& toggle(Pipeline* pipe);
    //void check_proposition(Pipeline* pipe);
    static State & getInstance(); // Get instance of the singleton class

private:
    // private constructor, copy constructor and = operator for the singleton class
    StartIdleState();
    StartIdleState(const StartIdleState& other);
    StartIdleState& operator=(const StartIdleState& other);

private:
    // Proposition evaluation functions
    bool check_ready_to_go(Pipeline* pipe);

};

#endif // START_IDLE_STATE_H
