#ifndef MID_PLANNER_PIPELINE_H
#define MID_PLANNER_PIPELINE_H
#include "cpc_motion_planning/pipeline.h"
#include <mid_plan/nf1based/mid_black_board.h>

class MidPlannerPipeline : public Pipeline
{

public:
  MidPlannerPipeline();

  // Preparation function called per cycle
  void prepare_cycle() override;

  // Finishing function called per cycle
  void finish_cycle() override;

public:
  MidBlackboard m_bb; // Blackboard for data communication

};

#endif // MID_PLANNER_PIPELINE_H
