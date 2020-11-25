#include "glb_plan/global_planner.h"
#include "tf/tf.h"
GlobalPlanner::GlobalPlanner():
  m_map_loaded(false),
  m_odom_received(false)
{
  m_glb_tgt_sub = m_nh.subscribe("/set_global_goal", 1, &GlobalPlanner::goal_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &GlobalPlanner::slam_odo_call_back, this);

  m_glb_path_pub = m_nh.advertise<cpc_motion_planning::path_action>("/global_path",1);

  m_map_loaded = load_c_map();
  perform_edt();

  m_map_pcl = PointCloud::Ptr(new PointCloud);
  m_map_pcl->header.frame_id = "/world";
  m_map_vis_pub = m_nh.advertise<PointCloud> ("/glb_map_vis", 1);
  prepare_map_pcl();


  m_path_pcl = PointCloud::Ptr(new PointCloud);
  m_path_pcl->header.frame_id = "/world";
  m_path_vis_pub = m_nh.advertise<PointCloud> ("/glb_path_vis", 1);

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

void GlobalPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  set_goal(CUDA_GEO::pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  if (m_map_loaded && m_odom_received)
  {
    m_glb_path = plan(m_goal,CUDA_GEO::pos(m_curr_pose.x,m_curr_pose.y,0));
    m_ps->set_path(m_glb_path);
    m_ps->smooth_path();
    m_glb_path = m_ps->get_path();
    show_glb_path();
    publish_glb_path();
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

bool GlobalPlanner::load_c_map()
{
  bool ok = m_c_map.Load();

  m_origin = CUDA_GEO::pos(-30,-30,0);
  m_step_width = 0.05f;
  m_width = 1000;
  m_height = 1000;

  m_a_map = new Astar(m_width,m_height,1);
  m_a_map->setMapSpecs(m_origin,m_step_width);

  m_c = new unsigned char[m_width*m_height];
  m_g = new int[m_width*m_height];
  m_h = new int[m_width*m_height];
  m_s = new int[m_width*m_height];
  m_t = new int[m_width*m_height];

  build_axis_aligned_map();
  return ok;
}

// This is currently the 2D version
void GlobalPlanner::perform_edt()
{
  for (int x=0;x<m_width; x++)
  {
    phase1(x);
  }

  for (int y=0;y<m_height; y++)
  {
    phase2(y);
  }

  for (int x=0; x<m_width; x++)
  {
    for (int y=0; y<m_height; y++)
    {
      m_a_map->getEdtMapPtr()[toid(x,y)].d = sqrtf(m_h[toid(x,y)]);
    }
  }
}




void GlobalPlanner::phase1(int x)
{
  // scan 1
  if (m_c[toid(x,0)] > 100)
    m_g[toid(x,0)]=0;
  else
    m_g[toid(x,0)] = 10000;

  for (int y=1; y<m_height; y++)
  {
    if (m_c[toid(x,y)] > 100)
      m_g[toid(x,y)]=0;
    else
      m_g[toid(x,y)] = 1 + m_g[toid(x,y-1)];
  }
  // scan 2
  for (int y=m_height-2; y>=0; y--)
  {
    if (m_g[toid(x,y+1)] < m_g[toid(x,y)])
    {
      m_g[toid(x,y)] = 1+m_g[toid(x,y+1)];
    }
  }
}

void GlobalPlanner::phase2(int y)
{
  int q=0;
  m_s[toid(0,y)] = 0;
  m_t[toid(0,y)] = 0;
  for (int u=1;u<m_width;u++)
  {
    while (q>=0 && f(m_t[toid(q,y)],m_s[toid(q,y)],y)
           > f(m_t[toid(q,y)],u,y))
    {
      q--;
    }
    if (q<0)
    {
      q = 0;
      m_s[toid(0,y)]=u;
    }
    else
    {
      int w = 1+sep(m_s[toid(q,y)],u,y);
      if (w < m_width)
      {
        q++;
        m_s[toid(q,y)]=u;
        m_t[toid(q,y)]=w;
      }
    }
  }
  for (int u=m_width-1;u>=0;u--)
  {
    m_h[toid(u,y)]=f(u,m_s[toid(q,y)],y);
    if (u == m_t[toid(q,y)])
      q--;
  }
}

std::vector<CUDA_GEO::pos> GlobalPlanner::plan(const CUDA_GEO::pos &goal_pos, const CUDA_GEO::pos &start_pos)
{
  std::vector<CUDA_GEO::pos> output;
  CUDA_GEO::coord start = m_a_map->pos2coord(start_pos);
  CUDA_GEO::coord goal = m_a_map->pos2coord(goal_pos);
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> coord_path =  m_a_map->AStar2D(goal,start,false,length);
  std::reverse(coord_path.begin(),coord_path.end());

  for (size_t i=0; i<coord_path.size();i++)
  {
    output.push_back(m_a_map->coord2pos(coord_path[i]));
  }

  return output;
}
