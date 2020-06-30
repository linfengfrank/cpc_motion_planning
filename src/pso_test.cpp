#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/pso/pso_planner.h>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/uav/uav_model.h>
#include <chrono>

typedef PSO::Planner<UAV::UAVModel, UAV::UAVJLTControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1>> Planner;
Planner *planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pso_test");
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
    planner = new Planner(50, 20, 4);
    planner->m_model.set_ini_state(initial_state);
    planner->m_model.set_var(sample_var);
    planner->m_model.set_limits(ctrl_limits);
    planner->m_eva.setTarget(target_state);
    float2 v_limit = make_float2(1.0, 0.5);
    float2 a_limit = make_float2(2.0, 1.0);
    float2 j_limit = make_float2(1.5, 1.5);
    planner->m_ctrl_dev.set_limit(v_limit, a_limit, j_limit);
    planner->m_ctrl_host.set_limit(v_limit, a_limit, j_limit);
    planner->initialize();

    /* fake map */
    CUDA_GEO::pos origin(-10,-10,-0.5);
    int3 edt_map_size = make_int3(100,100,10);
    EDTMap m_edt_map(origin,0.2,edt_map_size);
    m_edt_map.setup_device();

    planner->plan(m_edt_map);
    planner->release();
    float dt = PSO::PSO_CTRL_DT;
    float start_t = 0;
    float count_propagate_t = 0;

    UAV::UAVModel::State s;

    for (float t=0; t<10.0; t+=dt) {
        std::vector<UAV::UAVModel::State> traj = planner->generate_trajectory(start_t);
        if (traj.size() > 0) {
            s = traj[0];
        }
        printf("p: %.4f, %.4f, %.4f\n", s.p.x, s.p.y, s.p.z);
        printf("v: %.4f, %.4f, %.4f\n", s.v.x, s.v.y, s.v.z);
        printf("a: %.4f, %.4f, %.4f\n", s.a.x, s.a.y, s.a.z);
        start_t += dt;
        count_propagate_t += dt;
        if (count_propagate_t > planner->m_swarm.step_dt || traj.size() == 0) {
            count_propagate_t -= planner->m_swarm.step_dt;
            start_t = 0;
            ros::Time t1 = ros::Time::now();
            planner->m_model.set_ini_state(s);
            planner->initialize();
            planner->plan(m_edt_map);
            planner->release();
            ROS_INFO("cal time: %.2f", (ros::Time::now() - t1).toSec() * 1000.0);
        }
    }

    ros::spin();
    std::cout << "finish" << std::endl;
    return 0;
}
