#include "cpc_motion_planning/motion_planner.h"
#include "tf/tf.h"
#include <chrono>
MotionPlanner::MotionPlanner():
  m_received_map(false),
  m_received_state(false),
  m_edt_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &MotionPlanner::map_call_back, this);
  m_state_sub = m_nh.subscribe("/mavros/position/local", 1, &MotionPlanner::odo_call_back, this);

  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);

  m_planning_timer = m_nh.createTimer(ros::Duration(0.2), &MotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<5>(200,25);
  m_pso_planner->load_data_matrix();
  m_pso_planner->create_particles();

  m_display_planner = new PSO::Planner<5>(1,1);
  m_display_planner->load_data_matrix(true);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";
}

MotionPlanner::~MotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }

  m_pso_planner->free_data_matrix();
  m_pso_planner->delete_particles();
  delete m_pso_planner;

  m_display_planner->free_data_matrix(true);
  delete m_display_planner;
}

void MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_received_map)//!m_received_state ||
    return;

  double phi,theta,psi;

  //  tf::Quaternion q(m_odo.pose.pose.orientation.x,
  //                   m_odo.pose.pose.orientation.y,
  //                   m_odo.pose.pose.orientation.z,
  //                   m_odo.pose.pose.orientation.w);
  //  tf::Matrix3x3 m(q);
  //  m.getRPY(phi, theta, psi);

  static int icc = 0;

  PSO::State s;
  s.p.x = 0;//static_cast<float>((icc++)/5)*0.2;//m_odo.pose.pose.position.x;
  s.p.y = 0;//m_odo.pose.pose.position.y;
  s.s = 0;
  s.v = 0;//m_odo.twist.twist.linear.x + m_odo.twist.twist.linear.y;
  s.w = 0;
  s.theta = 0;

  PSO::State goal;
  goal.p = make_float2(10,10);
  auto start = std::chrono::steady_clock::now();
  m_pso_planner->plan(s,goal,*m_edt_map);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms" << std::endl;


  float dt = 0.1f;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO::PSO_STEP_DT));
    if (i > PSO::PSO_STEPS - 1)
      i = PSO::PSO_STEPS - 1;

    //std::cout<<i<<std::endl;
    float3 u = PSO::dp_control<5>(s, m_pso_planner->result.best_loc[i], m_display_planner->m_carrier, m_display_planner->m_ubc);
    PSO::model_forward(s,u,dt);

    pcl::PointXYZ clrP;
    clrP.x = s.p.x;
    clrP.y = s.p.y;
    clrP.z = 0.2f;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
}

void MotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_received_map = true;
  if (m_edt_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->setup_device();
  }
  else
  {
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
  }
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
}

void MotionPlanner::odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_received_state = true;
  m_odo = *msg;
}
