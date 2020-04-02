#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
#include <chrono>

float evaluate_trajectory_2(const PSO::State &s0, const PSO::State &goal, const PSO::Trace &tr, VoidPtrCarrier<5> ptr_car)
{
  PSO::State s = s0;
  float cost = 0;
  float dt = 0.05f;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO::PSO_STEP_DT));
    if (i > PSO::PSO_STEPS - 1)
      i = PSO::PSO_STEPS - 1;

    float3 u = PSO::dp_control<5>(s, tr[i], ptr_car);
    PSO::model_forward(s,u,dt);
    cost += PSO::process_cost(s,goal);
    printf(">>%f\n",PSO::process_cost(s,goal));
  }
  cost += PSO::final_cost(s,goal);
  return cost;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion_planning");
  PSO::Planner<5> p(200);
  std::cout<<"Init"<<std::endl;
  p.load_data_matrix();
  p.create_particles();


  PSO::Planner<5> p_host(1);
  p_host.load_data_matrix(true);



  PSO::State s;
  s.p = make_float2(1.870182, 0.102118);
  PSO::State goal;
  goal.p = make_float2(10,-10);
  for (int ctt = 0; ctt<50; ctt++)
  {
    //auto start = std::chrono::steady_clock::now();
    p.plan(s, goal);
    printf("%f %f\n",s.p.x,s.p.y);







//    float a = evaluate_trajectory_2(s, goal, p.result.best_loc, p_host.m_carrier);
//    printf("%f\n",a);
    //printf("%f %f, %f\n",p.result.best_loc[0].x,p.result.best_loc[0].y);
//        for(int i=0; i<2; i++)
//        {
//          printf("%f, %f\n",p.result.best_loc[i].x,p.result.best_loc[i].y);
//        }
        //printf("%f\n",p.result.best_cost);


    for (int k=0;k<4;k++)
    {
      float3 u = PSO::dp_control<5>(s, p.result.best_loc[0], p_host.m_carrier);
      PSO::model_forward(s,u,0.05f);
//      printf("%f, %f\n", s.s, s.v);

    }



    //    auto end = std::chrono::steady_clock::now();
    //    std::cout << "Consumed: "
    //              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    //              << "ms" << std::endl;
  }

  cudaDeviceSynchronize();
  std::cout<<"Free"<<std::endl;
  p.free_data_matrix();
  p.delete_particles();






  p_host.free_data_matrix(true);


  std::cout<<"Finish"<<std::endl;
  //ros::spin();
  return 0;
}
