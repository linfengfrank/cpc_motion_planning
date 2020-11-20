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
  p.load_c_map("/home/sp/cpc_ws/src/cpc_core_module/cpc_motion_planning/include/glb_plan/changhong_level_16.bmp");
  p.perform_edt();
    auto start_time = std::chrono::steady_clock::now();
  std::vector<CUDA_GEO::pos> path = p.plan(CUDA_GEO::pos(-15,-20,0),CUDA_GEO::pos(7,-1,0));

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "glb planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  PointCloud::Ptr m_pclOut;
  m_pclOut = PointCloud::Ptr(new PointCloud);
  m_pclOut->header.frame_id = "/world";

  //write the path out
  std::ofstream mylog;
  mylog.open("/home/sp/path.txt");
  for (size_t i=0; i<path.size();i++)
  {
    mylog<<path[i].x<<" "<<path[i].y<<std::endl;
  }
  mylog.close();



  ros::NodeHandle m_nh;
  ros::Publisher m_pc_pub=m_nh.advertise<PointCloud> ("/glb_map_vis", 1);

  while (ros::ok()) {


  CUDA_GEO::pos pnt;
  for (int x=0;x<p.m_width;x+=1)
  {
    for (int y=0;y<p.m_height;y+=1)
    {
      int idx = p.toid(x,y);
      int d_c = p.m_h[idx];
      if (d_c > 255) d_c = 255;
      int d = 255 - static_cast<int>(d_c);
      CUDA_GEO::pos pnt = p.m_a_map->coord2pos(CUDA_GEO::coord(x,y,0));
      pcl::PointXYZRGB clrP;
      clrP.x = pnt.x;
      clrP.y = pnt.y;
      clrP.z = 0;

      if ( d < 128)
      {
        clrP.b = 255-2*static_cast<unsigned char>(d);
        clrP.g = 2*static_cast<unsigned char>(d);
      }
      else
      {
        clrP.g = 255 - 2*(static_cast<unsigned char>(d) - 128);
        clrP.r = 2*(static_cast<unsigned char>(d)-128);
      }
      clrP.a = 255;
      m_pclOut->points.push_back (clrP);
    }
  }


  for (size_t i=0; i<path.size();i++)
  {
    pcl::PointXYZRGB clrP;
    clrP.x = path[i].x;
    clrP.y = path[i].y;
    clrP.z = 0;
    clrP.r = 255;
    clrP.g = 255;
    clrP.b = 255;
    clrP.a = 255;
    m_pclOut->points.push_back (clrP);
  }






  pcl_conversions::toPCL(ros::Time::now(), m_pclOut->header.stamp);
  m_pc_pub.publish (m_pclOut);
  m_pclOut->clear();
  ros::spinOnce();
  ros::Duration(2.1).sleep();
  }




  return 0;
}
