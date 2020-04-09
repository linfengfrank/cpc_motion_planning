#include <ros/ros.h>
#include <cpc_motion_planning/motion_planner.h>
#include <chrono>

//float RandomFloat(float a, float b) {
//    float random = ((float) rand()) / (float) RAND_MAX;
//    float diff = b - a;
//    float r = random * diff;
//    return a + r;
//}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion_planning");



  MotionPlanner p;
  ros::spin();



//  CUDA_GEO::pos origin(0,0,0);
//  int3 edt_map_size = make_int3(100,100,10);
//   EDTMap m_edt_map(origin,0.2,edt_map_size);
//   m_edt_map.setup_device();


//  PSO::Planner<5> p(150,80);
//  std::cout<<"Init"<<std::endl;
//  p.load_data_matrix();
//  p.create_particles();

//  PSO::Planner<5> p_host(1,1);
//  p_host.load_data_matrix(true);

//  PSO::State s;
//  s.p = make_float2(20.0f,0);
//  s.s = 0;
//  s.theta = 0;
//  s.v = 1.50f;
//  s.w = 0;
//  PSO::State goal;
//  goal.p = make_float2(-20, 0);
//  for (int ctt = 0; ctt<1; ctt++)
//  {

////    auto start = std::chrono::steady_clock::now();
////    s.s = 0;
////    p.plan(s, goal,m_edt_map);
////    auto end = std::chrono::steady_clock::now();
////    std::cout << "Consumed: "
////              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
////              << "ms" << std::endl;


//       p.result.best_loc[0].x=-8.54f;
//       p.result.best_loc[1].x=-9.0f;
//       p.result.best_loc[2].x=0.0665709f;


////    float val = PSO::evaluate_trajectory_wrapper<5>(s, goal, p.result.best_loc, p_host.m_carrier, p_host.m_ubc,
////                              m_edt_map, p.result.best_loc);

//    p.result.best_loc[0].x = 2.5f;
//    float val2 = PSO::evaluate_trajectory_wrapper<5>(s, goal, p.result.best_loc, p_host.m_carrier, p_host.m_ubc,
//                              m_edt_map, p.result.best_loc);

//   // std::cout<<val<<" "<<val2<<std::endl;

////    for (int k=0;k<4;k++)
////    {
////      float3 u = PSO::dp_control<5>(s, p.result.best_loc[0], p_host.m_carrier, p_host.m_ubc);
////      PSO::model_forward(s,u,0.05f);
////      printf("%f %f %f %f\n",s.p.x,s.p.y,s.v,s.w);

////    }
//    std::cout<<"-------------------"<<std::endl;
//  }
//  cudaDeviceSynchronize();
//  std::cout<<"Free"<<std::endl;
//  p.free_data_matrix();
//  p.delete_particles();
//  p_host.free_data_matrix(true);
//  std::cout<<"Finish"<<std::endl;

  return 0;
}
