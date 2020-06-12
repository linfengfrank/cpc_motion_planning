#include <ros/ros.h>
#include <cpc_motion_planning/uav_motion_planner.h>
#include <cpc_motion_planning/uav_nf1_motion_planner.h>
#include <chrono>

float RandomFloat(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion_planning");



  //  UAVMotionPlanner p;
  //  ros::spin();



  CUDA_GEO::pos origin(0,0,0);
  int3 edt_map_size = make_int3(100,100,10);
  EDTMap m_edt_map(origin,0.2,edt_map_size);
  m_edt_map.setup_device();

  UAV::UAVModel::State s;
  s.p = make_float3(-2.5,1.5,0);


  float dt = 0.05;

  PSO::Planner<CORRID_UAV> p(100,40,1);
  p.initialize(false);

  PSO::Planner<CORRID_UAV> q(1,1,1);
  q.initialize(true);


  //--------------------------------------------------------------------
  std::ifstream corridor_file;
  corridor_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/in.txt");
  std::vector<float3> way_points;
  std::vector<float3> sfc_dists;
  float data[6];

  //---
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5])
    {
      way_points.push_back(make_float3(data[0],data[1],data[2]));
      sfc_dists.push_back(make_float3(data[3],data[4],data[5]));
      std::cout<<data[0]<<" "<<data[1]<<" "<<data[2]<<" "<<data[3]<<" "<<data[4]<<" "<<data[5]<<std::endl;
    }
    else
    {
      break;
    }
  }

  //---
  std::cout<<"Constructing the corridor"<<std::endl;
  p.m_eva.m_sfc.n = way_points.size()-1;
  UAV::CorridorEvaluator::Corridor *sfc=  new UAV::CorridorEvaluator::Corridor[p.m_eva.m_sfc.n];
  for (uint i=0; i<way_points.size()-1; i++)
  {
    sfc[i].set_data(way_points[i],way_points[i+1],sfc_dists[i].x);
  }
  CUDA_ALLOC_DEV_MEM(&p.m_eva.m_sfc.c,sizeof(UAV::CorridorEvaluator::Corridor)*p.m_eva.m_sfc.n);
  CUDA_MEMCPY_H2D(p.m_eva.m_sfc.c,sfc,sizeof(UAV::CorridorEvaluator::Corridor)*p.m_eva.m_sfc.n);

  delete [] sfc;
  std::cout<<"There are "<<p.m_eva.m_sfc.n<<" corridors."<<std::endl;

  //--------------------------------------------------------------------


  p.m_model.set_ini_state(s);
  q.m_model.set_ini_state(s);
  //  m_pso_planner->m_eva.m_curr_pos = s.p;
  //  m_pso_planner->m_eva.m_curr_yaw = m_yaw_state.p;
  //  m_ref_gen_planner->set_problem(s,m_goal);

  std::ofstream log_file;
  log_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/out.txt");
  for (int ctt = 0; ctt<100; ctt++)
  {
    auto start = std::chrono::steady_clock::now();
    p.plan(m_edt_map);

    auto end = std::chrono::steady_clock::now();
    std::cout << "Consumed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms, cost: " << p.result.best_cost<<std::endl;


    std::vector<UAV::UAVModel::State> traj = q.generate_trajectory(p.result.best_loc);

    for (unsigned int i=0;i<5;i++)
    {
      s = traj[i];
      p.m_model.set_ini_state(s);
      q.m_model.set_ini_state(s);
      // std::cout<<p.result.best_cost<<std::endl;
      // std::cout<<s.p.x<<" "<<s.p.y<<" "<<s.p.z<<" "<<p.result.best_cost<<std::endl;
      log_file<<s.p.x<<" "<<s.p.y<<" "<<s.p.z<<" "<<s.v.x<<" "<<s.v.y<<" "<<s.v.z<<" "<<s.a.x<<" "<<s.a.y<<" "<<s.a.z<<" "<<p.result.best_cost<<std::endl;
    }
  }
  log_file.close();

  CUDA_FREE_DEV_MEM(p.m_eva.m_sfc.c);
  std::cout<<"Finish"<<std::endl;

  return 0;
}
