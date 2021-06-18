#include <ros/ros.h>
#include <glb_plan/global_planner.h>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "edt/gpu_edt.cuh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "glb_planning");
  GlobalPlanner p;
  ros::spin();
  return 0;
}
