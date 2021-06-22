#include "glb_plan/global_planner.h"
#include "tf/tf.h"
GlobalPlanner::GlobalPlanner():
  m_glb_path_id(0),
  m_map_loaded(false),
  m_odom_received(false),
  m_home_position(make_float2(0,0)),
  m_auto_mission_started(false)
{
  m_nh.param<float>("/global_safety_radius",m_safety_radius,0.51f);
  std::vector<float> home_loc;
  m_nh.getParam("/home_position",home_loc);
  if(home_loc.size() == 2)
    m_home_position = make_float2(home_loc[0], home_loc[1]);

  m_glb_tgt_sub = m_nh.subscribe("/set_global_goal", 1, &GlobalPlanner::goal_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &GlobalPlanner::slam_odo_call_back, this);
  m_glb_plan_execute_sub = m_nh.subscribe("/exe_glb_plan",1,&GlobalPlanner::exe_curr_glb_plan, this);
  m_go_home_sub = m_nh.subscribe("/return_home", 1, &GlobalPlanner::go_home, this);
  m_change_map_sub = m_nh.subscribe("/cpc_cmd",1,&GlobalPlanner::change_map, this);

  m_glb_path_pub = m_nh.advertise<cpc_motion_planning::path>("/global_path",1);

  // Load map for planning
  boost::shared_ptr<nav_msgs::OccupancyGrid const> glb_map_ptr;
  glb_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/for_planning/map");
  m_c_map = *glb_map_ptr;
  prepare_c_map();
  perform_edt();
  m_map_loaded = true;

  m_map_pcl = PointCloud::Ptr(new PointCloud);
  m_map_pcl->header.frame_id = "/world";
  m_map_vis_pub = m_nh.advertise<PointCloud> ("/glb_map_vis", 1);
  prepare_map_pcl();


  m_path_pcl = PointCloud::Ptr(new PointCloud);
  m_path_pcl->header.frame_id = "/world";
  m_path_vis_pub = m_nh.advertise<PointCloud> ("/glb_path_vis", 1);

  m_glb_plan_srv =  m_nh.advertiseService("/glb_plan_srv",&GlobalPlanner::glb_plan_service,this);
  m_recorded_path_srv = m_nh.advertiseService("/exe_recorded_path", &GlobalPlanner::exe_recorded_path, this);

  m_show_map_timer = m_nh.createTimer(ros::Duration(2.0), &GlobalPlanner::show_glb_map, this);

  m_ps = new PathSmoother(m_a_map);
}

GlobalPlanner::~GlobalPlanner()
{
  delete m_ps;
  delete m_a_map;
  delete [] m_c;
  delete [] m_g;
  delete [] m_h;
  delete [] m_s;
  delete [] m_t;
}

void GlobalPlanner::change_map(const std_msgs::String::ConstPtr &msg)
{
  // change the glb planner's map
  reset_planner();
  std::string map_info_str = msg->data;
  //m_map_loaded = read_c_map(map_info_str);
  prepare_c_map();
  perform_edt();
  prepare_map_pcl();
  m_ps->set_map(m_a_map);

  // call srv to change the map of aux_mapper
  cpc_aux_mapping::set_c_map srv;
  srv.request.map_info = map_info_str;
  m_aux_map_set_cmap_client.call(srv);

}

void GlobalPlanner::reset_planner()
{
  m_glb_path.clear();
  m_show_map_timer.start();
  m_map_loaded = false;
  m_odom_received = false;

  delete m_a_map;
  delete [] m_c;
  delete [] m_g;
  delete [] m_h;
  delete [] m_s;
  delete [] m_t;

  m_curr_act_path.clear();
  m_auto_mission_started = false;

}

void GlobalPlanner::exe_curr_glb_plan(const std_msgs::Bool::ConstPtr &msg)
{
  if (m_map_loaded && m_glb_path_msg.actions.size()>0)
  {
    publish_glb_path(m_glb_path_msg);
  }
}

bool GlobalPlanner::glb_plan_service(cpc_motion_planning::glb_plan_srv::Request &req,
                      cpc_motion_planning::glb_plan_srv::Response &res)
{
  m_glb_path_msg.actions.clear();
  CUDA_GEO::pos start(req.start.position.x, req.start.position.y,0);
  CUDA_GEO::pos goal(req.goal.position.x, req.goal.position.y,0);

  if (m_map_loaded)
  {
    m_glb_path = plan(goal,start);
    m_ps->set_path(m_glb_path);
    m_ps->smooth_path();
    m_glb_path = m_ps->get_path();

    geometry_msgs::Pose tmp_pose;
    for (CUDA_GEO::pos pnt: m_glb_path)
    {
      tmp_pose.position.x = pnt.x;
      tmp_pose.position.y = pnt.y;
      res.path.push_back(tmp_pose);
    }

    if (!req.save_as_name.empty())
    {
      std::ofstream mylog(req.save_as_name);
      for (CUDA_GEO::pos pnt: m_glb_path)
        mylog<<pnt.x<<" "<<pnt.y<<std::endl;

      mylog.close();
    }
    prepare_glb_path();
  }
  return true;
}

void GlobalPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_glb_path_msg.actions.clear();
  set_goal(CUDA_GEO::pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  if (m_map_loaded && m_odom_received)
  {
    m_glb_path = plan(m_goal,CUDA_GEO::pos(m_curr_pose.x,m_curr_pose.y,0));
    m_ps->set_path(m_glb_path);
    m_ps->smooth_path();
    m_glb_path = m_ps->get_path();
    show_glb_path();
    prepare_glb_path();
    publish_glb_path(m_glb_path_msg);
  }
}

void GlobalPlanner::set_goal(CUDA_GEO::pos goal)
{
  m_goal = goal;
}

void GlobalPlanner::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_odom_received = true;
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

void GlobalPlanner::prepare_c_map()
{
  m_origin = CUDA_GEO::pos(-35,-35,0);
  m_step_width = 0.05f;
  m_width = 1200;
  m_height = 1200;

  m_a_map = new Astar(m_width,m_height,1);
  m_a_map->setMapSpecs(m_origin,m_step_width);

  m_c = new unsigned char[m_width*m_height];
  m_g = new int[m_width*m_height];
  m_h = new int[m_width*m_height];
  m_s = new int[m_width*m_height];
  m_t = new int[m_width*m_height];

  build_axis_aligned_map();
}

std::vector<CUDA_GEO::pos> GlobalPlanner::plan(const CUDA_GEO::pos &goal_pos, const CUDA_GEO::pos &start_pos)
{
  std::vector<CUDA_GEO::pos> output;
  CUDA_GEO::coord start = m_a_map->pos2coord(start_pos);
  start = m_a_map->get_first_free_coord(start,m_safety_radius);
  CUDA_GEO::coord goal = m_a_map->pos2coord(goal_pos);
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> coord_path =  m_a_map->AStar2D(goal,start,false,length,m_safety_radius);
  std::reverse(coord_path.begin(),coord_path.end());

  for (size_t i=0; i<coord_path.size();i++)
  {
    output.push_back(m_a_map->coord2pos(coord_path[i]));
  }

  return output;
}

bool GlobalPlanner::exe_recorded_path(cpc_motion_planning::exe_recorded_path::Request &req,
                                      cpc_motion_planning::exe_recorded_path::Response &res)
{
  m_glb_path_msg.actions.clear();
  res.success = false;
  // Read in the data files
  std::ifstream corridor_file;
  float data[2];
  std::vector<float2> wps, wps_raw;
  corridor_file.open(req.file_name);
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1])
    {
      wps_raw.push_back((make_float2(data[0],data[1])));
    }
    else
    {
      break;
    }
  }

  // Scan the wps_raw to remove the stopped part
  if (wps_raw.size() <=1 )
    return true;

  wps.push_back(wps_raw.front());
  for (size_t i = 1; i < wps_raw.size(); i++)
  {
    float2 d = wps_raw[i] - wps.back();
    if (sqrtf(dot(d,d)) > 0.1)
      wps.push_back(wps_raw[i]);
  }

  // We need at least two waypoint to form a line
  if(wps.size()>1)
  {
    m_curr_act_path.clear();
    for (size_t i=0; i<wps.size()-1;i++)
    {
      m_curr_act_path = cascade_vector(m_curr_act_path,make_straight_path(wps[i],wps[i+1]));
    }
    std::vector<size_t> split_idx = split_merge(m_curr_act_path);

    std::vector<size_t> cusp_idx;
    cusp_idx.push_back(split_idx.front());
    for (size_t i=1; i<split_idx.size()-1;i++)
    {
      float2 a = m_curr_act_path[split_idx[i]] - m_curr_act_path[split_idx[i-1]];
      float2 b = m_curr_act_path[split_idx[i+1]] - m_curr_act_path[split_idx[i]];
      float norm_a = sqrtf(dot(a,a));
      float norm_b = sqrtf(dot(b,b));
      float theta = acosf(dot(a,b)/(norm_a*norm_b));

      if (theta > 0.5*M_PI)
        cusp_idx.push_back(split_idx[i]);
    }
    cusp_idx.push_back(split_idx.back());

    cpc_motion_planning::path glb_path;
    glb_path.request_ctt =  m_glb_path_id++;

    //--- set the path action
    for (size_t i=0; i<cusp_idx.size()-1;i++)
    {
      std::cout<<cusp_idx[i]<<" "<<cusp_idx[i+1]<<" "<<m_curr_act_path.size()-1<<std::endl;
      cpc_motion_planning::path_action pa;
      for (size_t j=cusp_idx[i]; j<cusp_idx[i+1];j++)
      {
        pa.x.push_back(m_curr_act_path[j].x);
        pa.y.push_back(m_curr_act_path[j].y);
        pa.theta.push_back(0);
        pa.type = cpc_motion_planning::path_action::TYPE_FORWARD;
      }
      glb_path.actions.push_back(pa);
    }
    m_glb_path_msg = glb_path;
  }

  res.success = true;
  return true;
}

void GlobalPlanner::go_home(const std_msgs::Bool::ConstPtr &msg)
{
  geometry_msgs::PoseStamped::Ptr tgt(new geometry_msgs::PoseStamped());
  tgt->pose.position.x = m_home_position.x;
  tgt->pose.position.y = m_home_position.y;
  tgt->pose.position.z = 0;
  goal_call_back(tgt);
}
