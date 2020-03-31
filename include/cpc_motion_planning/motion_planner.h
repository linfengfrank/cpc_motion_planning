#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H
#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
class MotionPlanner
{
public:
  MotionPlanner();
  ~MotionPlanner();

  void plan();

private:
    ros::NodeHandle nh;

};

#endif // MOTION_PLANNER_H
