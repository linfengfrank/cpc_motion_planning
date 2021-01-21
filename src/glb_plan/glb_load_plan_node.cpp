#include <ros/ros.h>
#include <glb_plan/global_plan_loader.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "glb_plan_loader");
  GlobalPlanLoader p;
  ros::spin();
  return 0;
}
