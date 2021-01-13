#include "mid_plan/nf1based/nf1_mid_planner.h"
#include "edt/gpu_edt.cuh"
#include "tf/tf.h"

NF1MidPlanner::NF1MidPlanner():
  m_received_map(false),
  m_received_goal(false),
  m_line_following_mode(false),
  m_curr_act_id(-1)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &NF1MidPlanner::map_call_back,this);
  m_glb_tgt_sub = m_nh.subscribe("/set_global_goal", 1, &NF1MidPlanner::goal_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &NF1MidPlanner::slam_odo_call_back, this);
  m_glb_path_sub = m_nh.subscribe("/global_path",1, &NF1MidPlanner::glb_path_call_back, this);
  m_goal_reach_sub = m_nh.subscribe("/target_reached",1, &NF1MidPlanner::goal_reached_call_back, this);


  m_pc_pub = m_nh.advertise<PointCloud> ("/nf1_vis", 1);
  m_mid_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);
  m_nf1_pub = m_nh.advertise<cpc_aux_mapping::nf1_task>("/nf1",1);
  m_straight_line_vis_pub = m_nh.advertise<visualization_msgs::Marker>("path_viz",1);

  m_pclOut = PointCloud::Ptr(new PointCloud);
  m_pclOut->header.frame_id = "/world";
  m_glb_plan_timer = m_nh.createTimer(ros::Duration(0.333), &NF1MidPlanner::plan, this);
}

NF1MidPlanner::~NF1MidPlanner()
{
  if (m_d_map)
  {
    delete m_d_map;
    delete m_a_map;
    delete m_line_map;

    for (int i=0;i<2;i++)
      cuttDestroy(m_rot_plan[i]);
  }
}

void NF1MidPlanner::goal_reached_call_back(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  int path_id = msg->data[0];
  int act_id = msg->data[1];
  if (path_id == m_path.request_ctt && act_id == m_curr_act_id
      && m_curr_act_id+1 < m_path.actions.size())
  {
      m_curr_act_id++;
      set_curr_act_path();
  }
}

void NF1MidPlanner::glb_path_call_back(const cpc_motion_planning::path::ConstPtr &msg)
{
  // Read in the data files
  m_line_following_mode = true;
  m_received_goal = true;
  m_path = *msg;
  m_curr_act_id = 0;
  set_curr_act_path();
}

void NF1MidPlanner::set_curr_act_path()
{
  cpc_motion_planning::path_action &pa = m_path.actions[m_curr_act_id];
  if (pa.x.size() > 0)
  {
    m_curr_act_path.clear();
    for (size_t i=0; i<pa.x.size(); i++)
    {
      m_curr_act_path.push_back(make_float2(pa.x[i],pa.y[i]));
    }
    float3 act_path_goal = make_float3(m_curr_act_path.back().x, m_curr_act_path.back().y, 0);
    set_nf1_task_info(pa.type, m_path.request_ctt, m_curr_act_id, act_path_goal);
    m_closest_pnt_idx = -1;
  }
  else
  {
    // jump to the next act
    if (m_curr_act_id+1 < m_path.actions.size())
    {
        m_curr_act_id++;
        set_curr_act_path();
    }
  }
}

void NF1MidPlanner::setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize)
{
  msg.x_origin = map->getOrigin().x;
  msg.y_origin = map->getOrigin().y;
  msg.z_origin = map->getOrigin().z;
  msg.width = map->getGridStep();

  if (resize)
  {
    msg.x_size = map->getMaxX();
    msg.y_size = map->getMaxY();
    msg.z_size = 1;
    msg.payload8.resize(sizeof(CostTheta)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}

void NF1MidPlanner::copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map)
{
  CUDA_GEO::coord c;
  CostTheta *tmp = static_cast<CostTheta*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int y=0;y<map->getMaxY();y++)
  {
    for (int x=0;x<map->getMaxX();x++)
    {
        c.x = x;
        c.y = y;
        c.z = 0;
        tmp[i].c=m_d_map->getCost2Come(c,0.0f);
        tmp[i].t=m_d_map->getTheta(c,0.0f);
        i++;
    }
  }
}

void NF1MidPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  m_received_map = true;
  CUDA_GEO::pos origin;
  if (m_d_map == nullptr)
  {
    m_d_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    setup_map_msg(m_nf1_msg.nf1,m_d_map,true);
    m_a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);

    //--- for line map init---
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_line_map = new EDTMap(origin,msg->width,edt_map_size);
    m_line_map->m_create_host_cpy = true;
    m_line_map->setup_device();

    int m = m_line_map->m_map_size.x;
    int n = m_line_map->m_map_size.y;
    int dim_0[2] = {m,n};
    int permu_0[2] = {1,0};
    int dim_1[2] = {n,m};
    int permu_1[2] = {1,0};
    cuttPlan(&m_rot_plan[0], 2, dim_0, permu_0, sizeof(int), nullptr);
    cuttPlan(&m_rot_plan[1], 2, dim_1, permu_1, sizeof(int), nullptr);
  }
  m_d_map->copyEdtData(msg);
  m_a_map->copyEdtData(msg);

  m_line_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
}

void NF1MidPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_received_goal= true;
  set_goal(CUDA_GEO::pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  float3 act_path_goal = make_float3(m_goal.x, m_goal.y, 0);
  set_nf1_task_info(cpc_aux_mapping::nf1_task::TYPE_FORWARD, 0, m_curr_act_id++, act_path_goal);
}

void NF1MidPlanner::set_goal(CUDA_GEO::pos goal)
{
  m_goal = goal;
}

void NF1MidPlanner::plan(const ros::TimerEvent&)
{
  if (!m_received_goal || !m_received_map || !m_received_odom)
    return;

  auto start_time = std::chrono::steady_clock::now();

  CUDA_GEO::coord tgt;
  CUDA_GEO::coord start(m_d_map->getMaxX()/2,m_d_map->getMaxY()/2,0);
  bool is_future_path_blocked = true;
  bool is_goal_in_view = true;
  bool is_goal_blocked = false;
  if(m_line_following_mode)
  {
    prepare_line_map(get_local_path(is_future_path_blocked,is_goal_in_view));

    CUDA_GEO::coord glb_tgt = m_d_map->pos2coord(m_goal);
    glb_tgt.z = 0;

    start = m_d_map->get_first_free_coord(start);
    tgt = m_d_map->find_available_target_with_line(start,glb_tgt,m_line_map);

    if(is_goal_in_view && !(tgt == glb_tgt))
      is_goal_blocked = true;
  }
  else
  {
    CUDA_GEO::coord glb_tgt = m_d_map->pos2coord(m_goal);
    glb_tgt.z = 0;
    glb_tgt = m_d_map->rayCast(start,glb_tgt).back();

    // find the target
    float length = 0.0f;
    std::vector<CUDA_GEO::coord> path = m_a_map->AStar2D(glb_tgt,start,false,length);
    tgt = path[0];
  }
  m_d_map->dijkstra2D_with_line_map(tgt,m_line_map,is_future_path_blocked);

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Middle planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms, is future path blocked: "<<is_future_path_blocked<< std::endl;

  // publish the nf1 and mid_goal
  CUDA_GEO::pos carrot = m_d_map->coord2pos(tgt);
  setup_map_msg(m_nf1_msg.nf1,m_d_map,false);
  copy_map_to_msg(m_nf1_msg.nf1,m_d_map);
  m_nf1_msg.carrot_x = carrot.x;
  m_nf1_msg.carrot_y = carrot.y;
  m_nf1_msg.carrot_theta = 0;
  m_nf1_pub.publish(m_nf1_msg);
  publish_mid_goal(tgt);

  // if the current goal is blocked, swith to the next path action
  if (is_goal_blocked && m_curr_act_id+1 < m_path.actions.size())
  {
    m_curr_act_id++;
    set_curr_act_path();
  }


  //show the nf1 map
#ifdef SHOWPC
  publishMap();
#endif


}

void NF1MidPlanner::prepare_line_map(const std::vector<float2> &path)
{
  m_line_map->clearOccupancy();
  for (size_t i = 0; i < path.size(); i++)
  {
    m_line_map->setOccupancy(path[i],true);
  }
  CUDA_MEMCPY_H2D(m_line_map->m_sd_map,m_line_map->m_hst_sd_map,m_line_map->m_byte_size);
  GPU_EDT::edt_from_occupancy(m_line_map, m_rot_plan);
  CUDA_MEMCPY_D2H(m_line_map->m_hst_sd_map,m_line_map->m_sd_map,m_line_map->m_byte_size);
}

void NF1MidPlanner::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_received_odom = true;
  m_curr_pose.x = msg->pose.pose.position.x;
  m_curr_pose.y = msg->pose.pose.position.y;

  double phi,theta,psi;
  tf::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  m_curr_pose.z = psi;
}

std::vector<float2> NF1MidPlanner::get_local_path(bool &is_future_path_blocked, bool&is_goal_in_view)
{
  float2 m_curr_pos = make_float2(m_curr_pose.x, m_curr_pose.y);
  float2 diff;
  float min_dist = 1e6;
  float dist;

  // first find the closest point on the path
  if(m_closest_pnt_idx == -1)
  {
    for (size_t i =0;i<m_curr_act_path.size();i++)
    {
      diff = m_curr_pos - m_curr_act_path[i];
      dist = dot(diff,diff);

      if (dist < min_dist)
      {
        min_dist = dist;
        m_closest_pnt_idx = i;
      }
    }
  }
  else
  {
    int start_idx = max(0,m_closest_pnt_idx-80);
    int end_idx = min(m_curr_act_path.size(),m_closest_pnt_idx+80);

    for (int i = start_idx;i<end_idx;i++)
    {
      diff = m_curr_pos - m_curr_act_path[i];
      dist = dot(diff,diff);

      if (dist < min_dist)
      {
        min_dist = dist;
        m_closest_pnt_idx = i;
      }
    }
  }

  // from the closest point to the path down the road
  is_goal_in_view = false;
  int end_idx = min(m_curr_act_path.size(),m_closest_pnt_idx+120);
  if (end_idx == static_cast<int>(m_curr_act_path.size()))
    is_goal_in_view = true;

  std::vector<float2> local_path;
  CUDA_GEO::pos p;
  for (int i = m_closest_pnt_idx; i< end_idx; i++)
  {
    p.x = m_curr_act_path[i].x;
    p.y = m_curr_act_path[i].y;
    if(m_d_map->isInside(p)) //&& !is_curvature_too_big(m_curr_act_path,m_closest_pnt_idx,i))
      local_path.push_back(m_curr_act_path[i]);
    else
      break;
  }

  // determine whether the future path is blocked
  CUDA_GEO::coord exam_crd;
  float exam_dist;
  is_future_path_blocked = false;
  for (float2 exam_p : local_path)
  {
    p.x = exam_p.x;
    p.y = exam_p.y;
    p.z = 0;
    exam_crd = m_d_map->pos2coord(p);
    exam_dist = m_d_map->getEdt(exam_crd,0);
    if (exam_dist < 0.51f)
    {
      is_future_path_blocked = true;
      break;
    }
  }
  // from the closest point to the path before
  // this part is reserved so that it can be used to selected as local target
  // so when the "down the road path" has no available target, we can choose a target from this part
  int start_idx = max(0,m_closest_pnt_idx-80);
  std::vector<float2> pre_path;
  for (int i = m_closest_pnt_idx-1; i>= start_idx; i--)
  {
    p.x = m_curr_act_path[i].x;
    p.y = m_curr_act_path[i].y;
    if(m_d_map->isInside(p)) //&& !is_curvature_too_big(m_curr_act_path,m_closest_pnt_idx,i))
      pre_path.push_back(m_curr_act_path[i]);
    else
      break;
  }
  // reverse the pre_path part because it is added in a reverse manner
  std::reverse(pre_path.begin(),pre_path.end());
  // cascade the pre_path and the local_path
  local_path = cascade_vector(pre_path,local_path);

  if (local_path.size()==0)
  {
    set_goal(CUDA_GEO::pos(m_curr_act_path[m_closest_pnt_idx].x,m_curr_act_path[m_closest_pnt_idx].y,0));
  }
  else
  {
    set_goal(CUDA_GEO::pos(local_path.back().x,local_path.back().y,0));
  }
  return local_path;
}
