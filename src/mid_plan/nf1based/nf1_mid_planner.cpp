#include "mid_plan/nf1based/nf1_mid_planner.h"

NF1MidPlanner::NF1MidPlanner():
  m_received_map(false),
  m_received_goal(false)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &NF1MidPlanner::map_call_back,this);
  m_glb_tgt_sub = m_nh.subscribe("/move_base_simple/goal", 1, &NF1MidPlanner::goal_call_back, this);

  m_pc_pub = m_nh.advertise<PointCloud> ("/nf1_vis", 1);
  m_mid_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);
  m_nf1_pub = m_nh.advertise<cpc_aux_mapping::grid_map>("/nf1",1);

  m_pclOut = PointCloud::Ptr(new PointCloud);
  m_pclOut->header.frame_id = "/world";
  m_glb_plan_timer = m_nh.createTimer(ros::Duration(0.333), &NF1MidPlanner::plan, this);
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
    setup_map_msg(m_nf1_map_msg,m_d_map,true);
    m_a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
  }
  m_d_map->copyEdtData(msg);
  m_a_map->copyEdtData(msg);
}

void NF1MidPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  set_goal(CUDA_GEO::pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
}

void NF1MidPlanner::set_goal(CUDA_GEO::pos goal)
{
  m_received_goal= true;
  m_goal = goal;
}

void NF1MidPlanner::plan(const ros::TimerEvent&)
{
  if (!m_received_goal || !m_received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // set the target and the goal
  CUDA_GEO::coord start(m_d_map->getMaxX()/2,m_d_map->getMaxY()/2,0);
  CUDA_GEO::coord glb_tgt = m_d_map->pos2coord(m_goal);
  glb_tgt.z = 0;
  glb_tgt = m_d_map->rayCast(start,glb_tgt).back();

  // find the target and construct nf1
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = m_a_map->AStar2D(glb_tgt,start,false,length);
  m_d_map->dijkstra2D(path[0]);

  // publish the nf1 and mid_goal
  setup_map_msg(m_nf1_map_msg,m_d_map,false);
  copy_map_to_msg(m_nf1_map_msg,m_d_map);
  m_nf1_pub.publish(m_nf1_map_msg);
  publish_mid_goal(path[0]);

  //show the nf1 map
#ifdef SHOWPC
  publishMap();
#endif

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Middle planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;
}
