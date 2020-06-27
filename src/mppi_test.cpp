#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/mppi/mppi_planner.h>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/uav/uav_model.h>
#include <chrono>

MPPI::Planner<UAV::UAVModel, UAV::UAVSIMPLEControl, UAV::SingleTargetEvaluator, UAV::UAVMppi<35>> *planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mppi_test");

    /* initial state */
    UAV::UAVModel::State initil_state;
    float3 sample_var = make_float3(0.5,0.5,0.5);

    /* Target */
    UAV::SingleTargetEvaluator::Target target_state;
    target_state.s.p.x = 1.0;
    target_state.s.p.y = -1.0;
    target_state.s.p.z = 1.0;

    /* planner initial */
    planner = new MPPI::Planner<UAV::UAVModel, UAV::UAVSIMPLEControl, UAV::SingleTargetEvaluator, UAV::UAVMppi<35>>(10);
    planner->m_model.set_ini_state(initil_state);
    planner->m_model.set_var(sample_var);
    planner->m_eva.setTarget(target_state);
    planner->initialize();
    // planner->plan();
    ros::spin();
    planner->release();
    std::cout << "finish" << std::endl;
    return 0;
}
