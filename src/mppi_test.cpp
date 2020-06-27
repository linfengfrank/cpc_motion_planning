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
    ros::NodeHandle nh;

    /* initial state */
    UAV::UAVModel::State initial_state;
    float3 sample_var = make_float3(0.05,0.05,0.05);
    float3 ctrl_limits = make_float3(2.0, 2.0, 1.0);

    /* Target */
    UAV::SingleTargetEvaluator::Target target_state;
    target_state.s.p.x = 1.0;
    target_state.s.p.y = -1.0;
    target_state.s.p.z = 1.0;
    target_state.oa = false;

    /* planner initial */
    planner = new MPPI::Planner<UAV::UAVModel, UAV::UAVSIMPLEControl, UAV::SingleTargetEvaluator, UAV::UAVMppi<35>>(640, 0.1);
    planner->m_model.set_ini_state(initial_state);
    planner->m_model.set_var(sample_var);
    planner->m_model.set_limits(ctrl_limits);
    planner->m_eva.setTarget(target_state);
    planner->initialize();

    /* fake map */
    CUDA_GEO::pos origin(-10,-10,-0.5);
    int3 edt_map_size = make_int3(100,100,10);
    EDTMap m_edt_map(origin,0.2,edt_map_size);
    m_edt_map.setup_device();

    /* simulator */
    UAV::UAVModel simulator;
    UAV::UAVModel::State sim_state = initial_state;
    simulator.set_ini_state(sim_state);

    for (float t = 0; t < 10.0; t += planner->m_core.step_dt) {
        ros::Time t1 = ros::Time::now();
        planner->plan(m_edt_map);
        float3 sim_u = planner->result.site[0];
        float3 rand_u = make_float3(0,0,0);
        planner->result.shift_update(rand_u);
        UAV::UAVMppi<35>::Trace n_cs;
        planner->m_core.set_nominal_and_initial_cs(n_cs, planner->result);
        ROS_INFO("cal time: %.2f", (ros::Time::now() - t1).toSec() * 1000.0);

        /* simulator propagation */
        printf("sim_u: %.2f, %.2f, %.2f\n", sim_u.x, sim_u.y, sim_u.z);
        simulator.model_forward(sim_state, sim_u, planner->m_core.step_dt);
        simulator.set_ini_state(sim_state);
        printf("p: %.2f, %.2f, %.2f\n", sim_state.p.x, sim_state.p.y, sim_state.p.z);
        printf("v: %.2f, %.2f, %.2f\n", sim_state.v.x, sim_state.v.y, sim_state.v.z);
        printf("a: %.2f, %.2f, %.2f\n", sim_state.a.x, sim_state.a.y, sim_state.a.z);

        planner->m_model.set_ini_state(sim_state);
    }

    ros::spin();
    planner->release();
    std::cout << "finish" << std::endl;
    return 0;
}
