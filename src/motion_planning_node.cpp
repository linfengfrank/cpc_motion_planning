#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_planning");
    PSO::Planner<5> p(100);
    std::cout<<"Init"<<std::endl;
    p.load_data_matrix();
    p.create_particles();

    p.plan();

    std::cout<<"Free"<<std::endl;
    p.free_data_matrix();
    p.delete_particles();

    cudaDeviceSynchronize();


//    p.load_data_matrix(true);
//    PSO::State s0;
//    PSO::Trace tr;
//    tr[0] = make_float3(2,2,0);
//    tr[1] = make_float3(2,2,0);
//    PSO::State s = s0;
//    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=PSO::PSO_dt)
//    {
//      int i = static_cast<int>(floor(t/PSO::PSO_STEP_DT));
//      if (i > PSO::PSO_STEPS - 1)
//          i = PSO::PSO_STEPS - 1;

//      float3 u = PSO::dp_control<5>(s, tr[i], p.m_carrier);
//      PSO::model_forward(s,u,0.05);
//      printf("%f %f\n",u.x,u.y);
//    }
//    p.free_data_matrix(true);


    std::cout<<"Finish"<<std::endl;
    //ros::spin();
    return 0;
}
