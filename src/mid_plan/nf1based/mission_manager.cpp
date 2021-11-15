#include "mid_plan/nf1based/mission_manager.h"

MissionManager::MissionManager()
{

}

MissionManager::~MissionManager()
{

}

void MissionManager::set_loiter_mission(const CUDA_GEO::pos &goal_pos)
{
  m_loiter_mission.act_id++;
  m_loiter_mission.m_loiter_goal = goal_pos;
  m_mission_type = LOITER;
}

void MissionManager::set_fullauto_mission(const cpc_motion_planning::path &path)
{
  m_fullauto_mission.path_id = path.request_ctt;
  m_fullauto_mission.act_id = 0;
  m_mission_type = FULL_AUTO_MISSION;
  m_fullauto_mission.m_path = path;
}
