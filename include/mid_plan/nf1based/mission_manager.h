#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include <ros/ros.h>
#include <mid_plan/utils/grid_graph.h>
#include <cpc_motion_planning/path.h>
#include <cpc_aux_mapping/nf1_task.h>
class MissionManager
{
public:
  enum TYPE
  {
    EMPTY,
    LOITER,
    FULL_AUTO_MISSION
  };

  struct LoiterMission
  {
    int path_id = cpc_aux_mapping::nf1_task::ID_LOITER;
    int act_id = 0;
    CUDA_GEO::pos m_loiter_goal;
  };

  struct FullAutoMission
  {
    int path_id = cpc_aux_mapping::nf1_task::ID_FIRST_MISSION;
    int act_id = 0;
    cpc_motion_planning::path m_path;
  };

public:
  MissionManager();
  ~MissionManager();
  void set_loiter_mission(const CUDA_GEO::pos &goal_pos);
  void set_fullauto_mission(const cpc_motion_planning::path &path);

private:
  TYPE m_mission_type = EMPTY;
  LoiterMission m_loiter_mission;
  FullAutoMission m_fullauto_mission;
};

#endif // MISSION_MANAGER_H
