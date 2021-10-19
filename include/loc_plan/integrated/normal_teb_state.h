#ifndef NOMARL_STATE_H
#define NOMARL_STATE_H
#include <loc_plan/integrated/state.h>
#include <iostream>

// Every state has to be a singleton class
class NormalTebState : public State
{
    // Propositoin enum
    // must start with 0 and end with the MAX_SIZE
    // cannot assign custom values
    enum PPS
    {
        STUCK = 0,
        REACH,
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
    NormalTebState();
    NormalTebState(const NormalTebState& other);
    NormalTebState& operator=(const NormalTebState& other);

private:
    // Proposition evaluation functions
    bool check_stuck(Pipeline* pipe);
    bool check_reach(Pipeline* pipe);
};

#endif // NOMARL_STATE_H
