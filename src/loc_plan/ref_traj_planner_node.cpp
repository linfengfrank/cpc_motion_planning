#include <loc_plan/ugv_reftraj_local_planner.h>

//#define REF_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::RefTrajEvaluator,UGV::UGVSwarm<4>

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion_planning");

  UGVRefTrajMotionPlanner p;
  ros::spin();
//  CUDA_GEO::pos origin(-10,-10,-1);
//  int3 edt_map_size = make_int3(100,100,30);
//  EDTMap m_edt_map(origin,0.2f,edt_map_size);
//  m_edt_map.setup_device();

//  PSO::Planner<REF_UGV> p(100,40,3);
//  p.initialize();

//  UGV::UGVModel::State s;
//  s.p = make_float2(2,0);
//  s.theta = 0.5*M_PI;
//  p.m_model.set_ini_state(s);

//  UGV::RefTrajEvaluator::Ref ref;
//  std::ofstream log_file;
//  log_file.open("/home/sp/Documents/out.txt");

//  float r = 2;
//  float theta = 0;
//  for(int ctt=0; ctt<250; ctt++)
//  {
//    for (int i=0; i<40; i++)
//    {
//      theta = 0.5*(ctt*0.2f + (i+1)*0.1f);
//      ref.tarj[i]=make_float3(r*cosf(theta),r*sinf(theta),theta+0.5*M_PI);
//    }
//    p.m_eva.setRef(ref);
//    auto start = std::chrono::steady_clock::now();
//    p.plan(m_edt_map);

//    auto end = std::chrono::steady_clock::now();
//    std::cout << "Consumed: "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//              << "ms, cost: " << p.result.best_cost<<std::endl;
//    std::vector<UGV::UGVModel::State> traj = p.generate_trajectory();
//    for (unsigned int i=0;i<4;i++)
//    {
//      s = traj[i];
//      p.m_model.set_ini_state(s);
//      theta = 0.5*(ctt*0.2f + (i+1)*0.05f);
//      // std::cout<<p.result.best_cost<<std::endl;
//      // std::cout<<s.p.x<<" "<<s.p.y<<" "<<s.p.z<<" "<<p.result.best_cost<<std::endl;
//      log_file<<s.p.x<<" "<<s.p.y<<" "<<s.theta<<" "<<p.result.best_cost<<" "<<r*cosf(theta)<<" "<<r*sinf(theta)<<" "<<theta+0.5*M_PI<<std::endl;
//    }
//  }
  std::cout<<"Finish"<<std::endl;

  return 0;
}
