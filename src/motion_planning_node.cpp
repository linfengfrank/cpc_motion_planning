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

//  PSO::Planner<5> p(200,25);
//  std::cout<<"Init"<<std::endl;
//  p.load_data_matrix();
//  p.create_particles();

//  PSO::Planner<5> p_host(1,1);
//  p_host.load_data_matrix(true);

//  PSO::State s;
//  s.p = make_float2(1.870182, 0.102118);
//  s.v = 0;
//  PSO::State goal;
//  goal.p = make_float2(10,-10);
//  for (int ctt = 0; ctt<2; ctt++)
//  {

//    auto start = std::chrono::steady_clock::now();
//    s.s = 0;
//    p.plan(s, goal);
//    auto end = std::chrono::steady_clock::now();
//    std::cout << "Consumed: "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//              << "ms" << std::endl;

//    for (int k=0;k<4;k++)
//    {
//      float3 u = PSO::dp_control<5>(s, p.result.best_loc[0], p_host.m_carrier, p_host.m_ubc);
//      PSO::model_forward(s,u,0.05f);
//      printf("%f %f %f %f\n",s.p.x,s.p.y,s.v,s.w);

//    }
//  }
//  cudaDeviceSynchronize();
//  std::cout<<"Free"<<std::endl;
//  p.free_data_matrix();
//  p.delete_particles();
//  p_host.free_data_matrix(true);
//  std::cout<<"Finish"<<std::endl;

  return 0;
}
