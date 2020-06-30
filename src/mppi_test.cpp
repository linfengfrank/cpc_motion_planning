#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/mppi/mppi_planner.h>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/uav/uav_jlt_control_mppi.h>
#include <chrono>

#define MPPI_STEP 35
typedef UAV::UAVMppi<MPPI_STEP>::Trace CtrlSeq;
typedef MPPI::Planner<UAV::UAVModel, UAV::UAVSIMPLEControl, UAV::SingleTargetEvaluator, UAV::UAVMppi<MPPI_STEP>> Planner;
Planner *planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mppi_test");
    ros::NodeHandle nh;

#ifdef TRAJ_VISUAL
    /* visual*/
    ros::Publisher sample_traj_pub = nh.advertise<PointCloud>("sample_traj", 1);
    ros::Publisher opt_traj_pub = nh.advertise<PointCloud>("opt_traj", 1);
#endif

    /* initial state */
    UAV::UAVModel::State initial_state;
    float3 sample_var = make_float3(0.3,0.3,0.3);
    float3 ctrl_limits = make_float3(2.0, 2.0, 1.0);

    /* Target */
    UAV::SingleTargetEvaluator::Target target_state;
    target_state.s.p.x = 5.0;
    target_state.s.p.y = -5.0;
    target_state.s.p.z = 1.0;
    target_state.oa = false;

    /* planner initial */
    planner = new Planner(2560, 1);
    planner->m_model.set_ini_state(initial_state);
    planner->m_model.set_var(sample_var);
    planner->m_model.set_limits(ctrl_limits);

    UAV::UAVJLTControl_MPPI jlt_control;
    float3 target_wp = target_state.s.p;
    float2 vM = make_float2(1.0, 0.5);
    float2 aM = make_float2(1.0, 0.5);
    float2 jM = make_float2(1.0, 0.7);
    jlt_control.set_limit(vM, aM, jM);
    CtrlSeq norm_csq = jlt_control.generate_control_sequence<UAV::UAVModel, UAV::UAVMppi<MPPI_STEP>>(
            planner->m_model, planner->m_core, target_wp);

//    norm_csq.print();

    planner->m_eva.setTarget(target_state);
    planner->initialize();

    /* fake map */
    CUDA_GEO::pos origin(-10,-10,-0.5);
    int3 edt_map_size = make_int3(100,100,10);
    EDTMap m_edt_map(origin,0.2,edt_map_size);
    m_edt_map.setup_device();

    planner->m_core.set_nominal_and_initial_cs(norm_csq,norm_csq);
    planner->plan(m_edt_map);
    float dt = MPPI::MPPI_CTRL_DT;
    float start_t = 0;
    ros::Rate rate(10.0);
    for (float t = 0; t < 10.0; t += dt) {
        std::vector<UAV::UAVModel::State> traj = planner->generate_trajectory(start_t);
        printf("p: %.4f, %.4f, %.4f\n", traj[0].p.x, traj[0].p.y, traj[0].p.z);
        printf("v: %.4f, %.4f, %.4f\n", traj[0].v.x, traj[0].v.y, traj[0].v.z);
//        printf("a: %.4f, %.4f, %.4f\n", traj[0].a.x, traj[0].a.y, traj[0].a.z);
        start_t += dt;
        if (planner->shift_update(dt)) {
            start_t = 0;
            ros::Time t1 = ros::Time::now();
            planner->m_model.set_ini_state(traj[0]);
            norm_csq = jlt_control.generate_control_sequence<UAV::UAVModel, UAV::UAVMppi<MPPI_STEP>>(
                    planner->m_model, planner->m_core, target_wp);
            planner->m_core.set_nominal_and_initial_cs(norm_csq, planner->result);
            planner->plan(m_edt_map);
            planner->visualize();
            ROS_INFO("cal time: %.2f", (ros::Time::now() - t1).toSec() * 1000.0);
#ifdef TRAJ_VISUAL
            std::cout << "sample num: " << planner->SampleOut->points.size() << std::endl;
            sample_traj_pub.publish(planner->SampleOut);
            opt_traj_pub.publish(planner->OptimalTrajOut);
#endif
        }

        rate.sleep();
    }

    ros::spin();
    planner->release();
    std::cout << "finish" << std::endl;
    return 0;
}
