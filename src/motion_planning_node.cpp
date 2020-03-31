#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_planning");
    PSO::Planner p(100);
    std::cout<<"Init"<<std::endl;
    p.load_data_matrix();
    p.create_particles();

    p.plan();

    std::cout<<"Free"<<std::endl;
    p.free_data_matrix();
    p.delete_particles();

    cudaDeviceSynchronize();
    std::cout<<"Finish"<<std::endl;
    //ros::spin();
    return 0;
}
