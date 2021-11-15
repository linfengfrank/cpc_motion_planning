#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include <cpc_motion_planning/state.h>
#include <iostream>

class MidPlannerPipeline;
class IdleState : public State
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
    IdleState();
    IdleState(const IdleState& other);
    IdleState& operator=(const IdleState& other);

private:
    MidPlannerPipeline *m_p = nullptr;

private:
    // Proposition evaluation functions
    bool check_ready_to_go();
};

#endif // IDLE_STATE_H
