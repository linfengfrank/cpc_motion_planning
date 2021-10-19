#ifndef STATE_MACHINE_BLACK_BOARD_H
#define STATE_MACHINE_BLACK_BOARD_H

#include <ros/ros.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_aux_mapping/nf1_task.h>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/ugv_nf1_evaluator.h>
#include <nav_msgs/Odometry.h>

class Blackboard
{
public:
    Blackboard()
    {
      //Add in the subscribers
      m_nf1_sub = m_nh.subscribe("/nf1",1,&Blackboard::nf1_call_back, this);
      m_map_sub = m_nh.subscribe("/edt_map", 1, &Blackboard::map_call_back, this);
      m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &Blackboard::raw_odo_call_back, this);
    #ifdef ADD_DELAY
      m_sub.subscribe(m_nh, "/slam_odom", 1);
      m_seq = new message_filters::TimeSequencer<nav_msgs::Odometry> (m_sub, ros::Duration(0.2), ros::Duration(0.01), 10);
      m_seq->registerCallback(&Blackboard::slam_odo_call_back, this);
    #else
      m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &Blackboard::slam_odo_call_back, this);
    #endif
    }

    //Things written on the balckboard:
public:
    bool m_goal_received = false;
    bool m_map_received = false;
    bool m_raw_odo_received = false;
    bool m_slam_odo_received = false;
    bool m_task_is_new = false;
    bool m_create_host_edt = false;
    UGV::NF1Evaluator::Target m_goal;
    UGV::UGVModel::State m_carrot;
    EDTMap *m_edt_map;
    NF1MapDT *m_nf1_map;
    nav_msgs::Odometry m_raw_odo, m_slam_odo;
    float m_ref_v = 0;
    float m_ref_w = 0;
    float m_ref_theta = 0;

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_map_sub;
    ros::Subscriber m_raw_odom_sub;
    ros::Subscriber m_slam_odom_sub;
    ros::Subscriber m_nf1_sub;
    uint8_t m_drive_dir;

private:
    void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg)
    {
      m_goal_received = true;
      // Setup the NF1 map
      if (m_nf1_map == nullptr)
      {
        CUDA_GEO::pos origin(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
        int3 m_nf1_map_size = make_int3(msg->nf1.x_size,msg->nf1.y_size,msg->nf1.z_size);
        m_nf1_map = new NF1MapDT(origin,msg->nf1.width,m_nf1_map_size);
        m_nf1_map->m_create_host_cpy = true;
        m_nf1_map->setup_device();
      }
      else
      {
        m_nf1_map->m_origin = CUDA_GEO::pos(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
        m_nf1_map->m_grid_step = msg->nf1.width;
      }
      CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));
      memcpy(m_nf1_map->m_hst_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));

      // Setup the drive type
      m_drive_dir = msg->drive_type;

      // Setup the goal
      if(m_goal.act_id != msg->act_id || m_goal.path_id != msg->path_id)
      {
        m_task_is_new = true;
        m_goal.s.p.x = msg->goal_x;
        m_goal.s.p.y = msg->goal_y;
        m_goal.s.theta = msg->goal_theta;
        m_goal.path_id = msg->path_id;
        m_goal.act_id = msg->act_id;
        m_goal.reaching_radius = 0.5f;
      }

      // Setup the carrot
      m_carrot.p.x = msg->carrot_x;
      m_carrot.p.y = msg->carrot_y;
      m_carrot.theta = msg->carrot_theta;
    }

    void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
    {
      m_map_received = true;
      if (m_edt_map == nullptr)
      {
        // Construct and initialize the edt map
        CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
        int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
        m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
        m_edt_map->m_create_host_cpy = m_create_host_edt;
        m_edt_map->setup_device();
      }
      else
      {
        // Set the edt map's origin location
        m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
        m_edt_map->m_grid_step = msg->width;
      }

      // Copy map data to the device
      CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));

      // Copy map data to the host
      if (m_create_host_edt)
        memcpy(m_edt_map->m_hst_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
    }

    void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
    {
      m_raw_odo_received = true;
      m_raw_odo = *msg;
    }

    void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
    {
      m_slam_odo_received = true;
      m_slam_odo = *msg;
    }

};

#endif // STATE_MACHINE_BLACK_BOARD_H
