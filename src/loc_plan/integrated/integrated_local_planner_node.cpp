#include <ros/ros.h>
#include <loc_plan/integrated/pipeline.h>
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



  Pipeline p;
  ros::spin();



//  CUDA_GEO::pos origin(0,0,0);
//  int3 edt_map_size = make_int3(100,100,10);
//  EDTMap m_edt_map(origin,0.2,edt_map_size);
//  m_edt_map.setup_device();

//  UGV::UGVModel::State s;


//  float dt = 0.05;
//   PSO::Planner<SIMPLE_UGV> p(1,1,1);
//   p.initialize(true);


//    for (int ctt = 0; ctt<100; ctt++)
//    {

//      float3 u = p.m_dp_ctrl.dp_control(s, make_float3(4,4,0));
//      p.m_model.model_forward(s,u,dt);

//      std::cout<<s.s<<" "<<s.theta<<std::endl;

//    }

    std::cout<<"Finish"<<std::endl;

  return 0;
}
