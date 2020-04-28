#include <ros/ros.h>
#include <cpc_motion_planning/motion_planner.h>
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



//  MotionPlanner p;
//  ros::spin();



  CUDA_GEO::pos origin(0,0,0);
  int3 edt_map_size = make_int3(100,100,10);
   EDTMap m_edt_map(origin,0.2,edt_map_size);
   m_edt_map.setup_device();


  PSO::Planner p(200,40,4);
  std::cout<<"Init"<<std::endl;
  p.load_data_matrix();
  p.create_particles();

  PSO::Planner p_host(1,1);
  p_host.load_data_matrix(true);

  PSO::State s;
  s.p = make_float3(0.0f,0,0);
  s.v = make_float3(0.0f,0,0);
  s.a = make_float3(0.0f,0,0);
  PSO::State goal;
  goal.p = make_float3(5,5,0);
  for (int ctt = 0; ctt<120; ctt++)
  {

    goal.p = make_float3(5+3*sin(ctt*0.1),5+3*cos(ctt*0.1),0);
    auto start = std::chrono::steady_clock::now();

    p.plan(s, goal,m_edt_map);
        auto end = std::chrono::steady_clock::now();
//        std::cout << "Consumed: "
//                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//                  << "ms" << std::endl;

//    for (int i=0; i< PSO::PSO_STEPS;i++)
//      printf("%f %f %f ", p.result.best_loc[i].x, p.result.best_loc[i].y, p.result.best_loc[i].z);

//    printf("%f\n", p.result.best_cost);




//       p.result.best_loc[0].x=0.200027;
//       p.result.best_loc[0].y=0.171891;
//       p.result.best_loc[0].z=0.191259;


//    float val = PSO::evaluate_trajectory_wrapper(s, goal, p.result.best_loc, p_host.m_carrier, p_host.m_ubc,
//                              m_edt_map, p.result.best_loc);
//    std::cout<<"-----------"<<std::endl;

//    p.result.best_loc[0].x=0;
//    p.result.best_loc[0].y=0;
//    p.result.best_loc[0].z=0;

////    p.result.best_loc[0].x = 2.5f;
//    float val2 = PSO::evaluate_trajectory_wrapper(s, goal, p.result.best_loc, p_host.m_carrier, p_host.m_ubc,
//                              m_edt_map, p.result.best_loc);

//    std::cout<<val<<" "<<val2<<std::endl;

    for (int k=0;k<4;k++)
    {
      float3 u = PSO::dp_control(s, p.result.best_loc[0], p_host.m_carrier, p_host.m_ubc);
      PSO::model_forward(s,u,0.05f);
      printf("%f %f %f %f ",s.p.x,s.v.x,s.a.x,u.x);
      printf("%f %f %f %f ",s.p.y,s.v.y,s.a.y,u.y);
      printf("%f %f %f %f ",s.p.z,s.v.z,s.a.z,u.z);
      printf("\n");
     // s.p.x += RandomFloat(-0.2,0.2);
     // printf("%f %f %f\n", p.result.best_loc[0].x, p.result.best_loc[0].y, p.result.best_loc[0].z);

    }
    //std::cout<<"-------------------"<<std::endl;
  }
  cudaDeviceSynchronize();
  std::cout<<"Free"<<std::endl;
  p.free_data_matrix();
  p.delete_particles();
  p_host.free_data_matrix(true);
  std::cout<<"Finish"<<std::endl;

  return 0;
}
