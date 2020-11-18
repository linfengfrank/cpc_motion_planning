#include <ros/ros.h>
#include <mid_plan/nf1based/nf1_mid_planner.h>
#include <chrono>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nf1_planning");
  NF1MidPlanner p;
  ros::spin();
  return 0;
}
