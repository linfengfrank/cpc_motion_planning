#include "cpc_motion_planning/motion_planner.h"
#include "tf/tf.h"
#include <chrono>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

MotionPlanner::MotionPlanner():
  m_received_map(false),
  m_raw_odo_received(false),
  m_slam_odo_received(false),
  m_goal_received(false),
  m_edt_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &MotionPlanner::map_call_back, this);
  m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &MotionPlanner::raw_odo_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &MotionPlanner::slam_odo_call_back, this);
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&MotionPlanner::goal_call_back, this);
  m_curr_ref_sub = m_nh.subscribe("/current_ref",1,&MotionPlanner::curr_ref_call_back, this);

  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &MotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner(150,60);
  m_pso_planner->load_data_matrix();
  m_pso_planner->create_particles();

  m_display_planner = new PSO::Planner(1,1);
  m_display_planner->load_data_matrix(true);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";

  m_ref_v = 0.0f;
  m_ref_w = 0.0f;

  //Initialize the control message
  m_ref_msg.rows = 2;
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

void MotionPlanner::curr_ref_call_back(const cpc_motion_planning::ref_data::ConstPtr &msg)
{
  m_ref_v = msg->data[0];
  m_ref_w = msg->data[1];
}

void MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_slam_odo_received || !m_raw_odo_received || !m_received_map || !m_goal_received)
    return;

  double phi,theta,psi;

  tf::Quaternion q(m_slam_odo.pose.pose.orientation.x,
                   m_slam_odo.pose.pose.orientation.y,
                   m_slam_odo.pose.pose.orientation.z,
                   m_slam_odo.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);

  PSO::State s;
  s.p.x = m_slam_odo.pose.pose.position.x;
  s.p.y = m_slam_odo.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;


  float v_err = m_ref_v-m_raw_odo.twist.twist.linear.x;
  float w_err = m_ref_w-m_raw_odo.twist.twist.angular.z;

  if (fabs(v_err) > 1.0 )
  {
    std::cout<<"------Reset v------"<<std::endl;
    s.v = m_raw_odo.twist.twist.linear.x + sgn<float>(v_err)*0.5;
  }
  else
  {
    s.v = m_ref_v;
  }

  if (fabs(w_err) > 1.0)
  {
    std::cout<<"------Reset w------"<<std::endl;
    s.w = m_raw_odo.twist.twist.angular.z + sgn<float>(w_err)*0.5;
  }
  else
  {
    s.w = m_ref_w;
  }

  //std::cout<<"v,w err: "<<v_err<<", "<<w_err<<std::endl;
  //  float yaw_diff = s.theta - m_goal.theta;
  //  yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;


  auto start = std::chrono::steady_clock::now();
  m_pso_planner->plan(s,m_goal,*m_edt_map);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms" << std::endl;

  std::cout<<s.p.x<<" "<<s.p.y<<" "<<s.s<<" "<<s.theta<<" "<<s.v<<" "<<s.w<<" "<<std::endl;
  std::cout<<"v,w err: "<<v_err<<", "<<w_err<<std::endl;
  std::cout<<"--------------"<<std::endl;

  //PSO::State cmd_state;
  float dt = PSO::PSO_CTRL_DT;
  int cols = 0;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO::PSO_STEP_DT));
    if (i > PSO::PSO_STEPS - 1)
      i = PSO::PSO_STEPS - 1;

    float3 u = PSO::dp_control(s, m_pso_planner->result.best_loc[i], m_display_planner->m_carrier, m_display_planner->m_ubc);
    PSO::model_forward(s,u,dt);

    pcl::PointXYZ clrP;
    clrP.x = s.p.x;
    clrP.y = s.p.y;
    clrP.z = 0.2f;
    m_traj_pnt_cld->points.push_back(clrP);

    m_ref_msg.data.push_back(s.v);
    m_ref_msg.data.push_back(s.w);
    cols++;
  }

  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();

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

void MotionPlanner::raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_raw_odo_received = true;
  m_raw_odo = *msg;
}

void MotionPlanner::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_slam_odo_received = true;
  m_slam_odo = *msg;
}

void MotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.p.x = msg->pose.position.x;
  m_goal.p.y = msg->pose.position.y;

  double phi,theta,psi;

  tf::Quaternion q( msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);



  m_goal.theta = psi;
}
