#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
#include <cpc_aux_mapping/grid_map.h>
#include <nav_msgs/Odometry.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpc_motion_planning/ref_data.h>
#include <cpc_motion_planning/JLT.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_single_target_evaluator.h>
#include <cpc_motion_planning/ugv/controller/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/controller/ugv_jlt_control.h>
#include <cpc_motion_planning/ugv/swarm/ugv_swarm.h>
#include <chrono>

#define CORRID_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::CorridorEvaluator,UGV::UGVSwarm<3>
float RandomFloat(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion_planning");



  //  UGVMotionPlanner p;
  //  ros::spin();



  CUDA_GEO::pos origin(0,0,0);
  int3 edt_map_size = make_int3(100,100,10);
  EDTMap m_edt_map(origin,0.2,edt_map_size);
  m_edt_map.setup_device();

  UGV::UGVModel::State s;
  s.p = make_float2(-2.5,1.5);
  s.theta = -M_PI*0.0f;


  float dt = 0.05;

  PSO::Planner<CORRID_UGV> p(100,40,3);
  p.initialize();


  //--------------------------------------------------------------------
  std::ifstream corridor_file;
  corridor_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/in2.txt");
  std::vector<float3> way_points;
  std::vector<float3> sfc_dists;
  float data[6];

  //---
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5])
    {
      way_points.push_back(make_float3(data[0],data[1],0));
      sfc_dists.push_back(make_float3(data[3],data[4],0));
      std::cout<<data[0]<<" "<<data[1]<<" "<<data[2]<<" "<<data[3]<<" "<<data[4]<<" "<<data[5]<<std::endl;
    }
    else
    {
      break;
    }
  }

  //---
  std::cout<<"Constructing the corridor"<<std::endl;

  std::vector<UGV::CorridorEvaluator::Corridor> sfc;
  for (uint i=0; i<way_points.size()-1; i++)
  {
    UGV::CorridorEvaluator::Corridor tmp;
    tmp.set_data(way_points[i],way_points[i+1],sfc_dists[i].x);
    sfc.push_back(tmp);
  }

  std::cout<<"There are "<<sfc.size()<<" corridors."<<std::endl;

  //--------------------------------------------------------------------


  p.m_model.set_ini_state(s);
  //  m_pso_planner->m_eva.m_curr_pos = s.p;
  //  m_pso_planner->m_eva.m_curr_yaw = m_yaw_state.p;
  //  m_ref_gen_planner->set_problem(s,m_goal);

  int tgt_ctt = 0;
  p.m_eva.m_sfc.c = sfc[tgt_ctt];

  std::ofstream log_file;
  log_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/out.txt");
  for (int ctt = 0; ctt<300; ctt++)
  {
    auto start = std::chrono::steady_clock::now();
    p.plan(m_edt_map);

    auto end = std::chrono::steady_clock::now();
    std::cout << "Consumed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms, cost: " << p.result.best_cost<<std::endl;


    std::vector<UGV::UGVModel::State> traj = p.generate_trajectory();

    for (unsigned int i=0;i<5;i++)
    {
      s = traj[i];
      p.m_model.set_ini_state(s);
      // std::cout<<p.result.best_cost<<std::endl;
      // std::cout<<s.p.x<<" "<<s.p.y<<" "<<s.p.z<<" "<<p.result.best_cost<<std::endl;
      log_file<<s.p.x<<" "<<s.p.y<<" "<<s.theta<<" "<<p.result.best_cost<<std::endl;
    }

    float2 diff = s.p - make_float2(sfc[tgt_ctt].b.x,sfc[tgt_ctt].b.y);
    if (sqrtf(dot(diff,diff)) < 1.0f && tgt_ctt + 1 < sfc.size())
    {
      tgt_ctt++;
      p.m_eva.m_sfc.c = sfc[tgt_ctt];
    }

  }
  log_file.close();

  std::cout<<"Finish"<<std::endl;

  return 0;
}
