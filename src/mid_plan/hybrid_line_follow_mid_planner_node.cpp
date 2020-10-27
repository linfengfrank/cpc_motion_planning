#include <ros/ros.h>
#include <mid_plan/grid_graph.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <algorithm>
#include "cpc_motion_planning/ref_data.h"
#include <mid_plan/hybrid_dijkstra.h>
#include <mid_plan/a_star.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#define SHOWPC
//#define DEBUG_COST
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* nf1_pub;
ros::Publisher* pc_pub;
ros::Publisher* path_pub;
ros::Publisher* mid_goal_pub;
ros::Publisher* vis_pub;

Dijkstra *mid_map=nullptr;
Astar *a_map=nullptr;

float FLY_HEIGHT;
bool first_run = true;
CUDA_GEO::pos curr_target_pos;
bool stucked = false;
bool received_cmd = false;
bool received_map = false;
bool received_ref = false;
CUDA_GEO::pos goal;
ros::Timer glb_plan_timer;
cpc_motion_planning::ref_data ref;
cpc_aux_mapping::grid_map nf1_map_msg;
float3 curr_pose;
ros::Publisher* glb_goal_pub;
//---
struct line_seg
{
  float2 a;
  float2 b;
  float2 uni;
  float tht;
  float dist;
  line_seg(float2 a_, float2 b_):a(a_),b(b_)
  {
    dist = sqrtf(dot(b-a,b-a));

    if (dist > 1e-6)
      uni = (b-a)/dist;
    else
      uni = make_float2(0,0);

    tht = atan2f(uni.y,uni.x);
  }
};
std::vector<line_seg> lines;
int current_line_id;
//---
void publishMap(int tgt_height_coord)
{
  //publish the point cloud to rviz for checking
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  for (int x=0;x<mid_map->getMaxX();x++)
  {
    for (int y=0;y<mid_map->getMaxY();y++)
    {
      for (int z=0;z<THETA_GRID_SIZE;z++)
      {
        c.x = x;
        c.y = y;
        c.z = z;
        float d_c = mid_map->m_getCost2Come(c,0.0f)*15;
        if (d_c > 255) d_c = 255;
        int d = static_cast<int>(d_c);

        //                std::cout<<d<<" ";
        //int color = (EDTMap::MAX_RANGE*EDTMap::MAX_RANGE-d)*255/EDTMap::MAX_RANGE/EDTMap::MAX_RANGE;
        //if (d < 4.1)
        {
          p = mid_map->coord2pos(c);
          pcl::PointXYZRGB clrP;
          clrP.x = p.x;
          clrP.y = p.y;
          clrP.z = p.z;

          if ( d < 128)
          {
            clrP.b = 255-2*static_cast<unsigned char>(d);
            clrP.g = 2*static_cast<unsigned char>(d);
          }
          else
          {
            clrP.g = 255 - 2*(static_cast<unsigned char>(d) - 128);
            clrP.r = 2*(static_cast<unsigned char>(d)-128);
          }



          clrP.a = 255;

          pclOut->points.push_back (clrP);
        }

      }
    }
  }
  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);

  pc_pub->publish (pclOut);

  pclOut->clear();
}
//---


//---
void stuckCallback(const std_msgs::Bool::ConstPtr& msg)
{
  stucked = msg->data;
  std::cout<<"Stucked."<<std::endl;
}
//---
void setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize)
{
  msg.x_origin = map->getOrigin().x;
  msg.y_origin = map->getOrigin().y;
  msg.z_origin = map->getOrigin().z;
  msg.width = map->getGridStep();

  if (resize)
  {
    msg.x_size = map->getMaxX();
    msg.y_size = map->getMaxY();
    msg.z_size = THETA_GRID_SIZE;
    msg.payload8.resize(sizeof(float)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map,int tgt_height_coord)
{
  CUDA_GEO::coord c;
#ifdef DEBUG_COST
  std::ofstream mylog;
  mylog.open("/home/sp/test/dj.txt");
#endif
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int z=0;z<THETA_GRID_SIZE;z++)
  {
    for (int y=0;y<map->getMaxY();y++)
    {
      for (int x=0;x<map->getMaxX();x++)
      {
        c.x = x;
        c.y = y;
        c.z = z;
        float d_c = mid_map->m_getCost2Come(c,0.0);
        tmp[i++]=d_c;
#ifdef DEBUG_COST
        mylog<<d_c<<" ";
#endif
      }
    }
  }
#ifdef DEBUG_COST
  mylog.close();
#endif
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  if (mid_map == nullptr)
  {
    mid_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    mid_map->copyEdtData(msg);
    mid_map->set_neighbours();
    setup_map_msg(nf1_map_msg,mid_map,true);

    a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
    a_map->copyEdtData(msg);
  }
  else
  {
    mid_map->copyEdtData(msg);
    a_map->copyEdtData(msg);
  }
}
//---
void show_path(const std::vector<float2> &wps)
{
  // For visulization
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;

  for (float2 wp: wps)
  {
    geometry_msgs::Point pnt;
    pnt.x = wp.x;
    pnt.y = wp.y;
    pnt.z = 0;
    line_strip.points.push_back(pnt);
  }
  vis_pub->publish(line_strip);
}
//---
void load_mission_callback(const std_msgs::Int32::ConstPtr &msg)
{
  // Read in the data files
  std::ifstream corridor_file;
  float data[3];
  std::vector<float2> wps;

  corridor_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/in.txt");
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1]>>data[2])
    {
      wps.push_back((make_float2(data[0],data[1])));
      std::cout<<data[0]<<" "<<data[1]<<" "<<static_cast<int>(data[2])<<std::endl;
    }
    else
    {
      break;
    }
  }

  // We need at least two waypoint to form a line
  if(wps.size()>1)
  {
    lines.clear();
    for (size_t i = 0; i < wps.size()-1; i++)
    {
      lines.push_back(line_seg(wps[i],wps[i+1]));
    }
    received_cmd = true;
    current_line_id = 0;

    geometry_msgs::PoseStamped glb_goal;
    glb_goal.pose.position.x = lines[current_line_id].b.x;
    glb_goal.pose.position.y = lines[current_line_id].b.y;
    glb_goal.pose.position.z = 0;

    tf::Quaternion quat;
    quat.setRPY( 0, 0, lines[current_line_id].tht );
    tf::quaternionTFToMsg(quat, glb_goal.pose.orientation);

    glb_goal_pub->publish(glb_goal);

    show_path(wps);
  }
}
//---
void glb_plan(const ros::TimerEvent&)
{
  if (!received_cmd || !received_map)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // Set all points
  int tgt_height_coord = mid_map->calcTgtHeightCoord(FLY_HEIGHT);

  CUDA_GEO::coord start(mid_map->getMaxX()/2,mid_map->getMaxY()/2,mid_map->getMaxZ()/2);
  start.z = tgt_height_coord;
  CUDA_GEO::pos start_pos = mid_map->coord2pos(start);

  //---
  // Find the projection point on the line
  CUDA_GEO::pos goal(lines[current_line_id].b.x,lines[current_line_id].b.y,0);
  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;


  CUDA_GEO::pos proj_pnt;
  CUDA_GEO::pos seg_a(lines[current_line_id].a.x,lines[current_line_id].a.y,FLY_HEIGHT);
  CUDA_GEO::pos seg_b(lines[current_line_id].b.x,lines[current_line_id].b.y,FLY_HEIGHT);

  bool use_line = true;

  if (mid_map->lineseg_proj(seg_a,seg_b,start_pos,proj_pnt) < 0)
  {
    use_line = false;
  }


  if (mid_map->isInside(proj_pnt))
  {
    glb_tgt = mid_map->rayCast(mid_map->pos2coord(proj_pnt),glb_tgt,2.5f).back();
  }
  else
  {
    glb_tgt = mid_map->rayCast(start,mid_map->pos2coord(proj_pnt),2.5f).back();
    use_line = false;
  }

#ifdef DEBUG_COST
  static bool done = false;
  if (done) return;
  done = true;
#endif

  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = a_map->AStar2D(glb_tgt,start,false,length);

  if(!use_line)
    mid_map->hybrid_dijkstra_with_int_theta(path[0]);
  else
    mid_map->hybrid_dijkstra_with_int_theta_with_line(path[0],seg_a,seg_b);

  setup_map_msg(nf1_map_msg,mid_map,false);
  copy_map_to_msg(nf1_map_msg,mid_map,tgt_height_coord);
  nf1_pub->publish(nf1_map_msg);

  //Get the mid goal
  //std::reverse(path.begin(),path.end());
  unsigned int tgt_idx = a_map->findTargetCoord(path);
  geometry_msgs::PoseStamped mid_goal_pose;
  CUDA_GEO::pos mid_goal_pos = mid_map->coord2pos(path[tgt_idx]);
  mid_goal_pose.header.frame_id="world";
  mid_goal_pose.pose.position.x = mid_goal_pos.x;
  mid_goal_pose.pose.position.y = mid_goal_pos.y;
  mid_goal_pose.pose.position.z = mid_goal_pos.z;
  mid_goal_pub->publish(mid_goal_pose);

#ifdef SHOWPC
  publishMap(tgt_height_coord);
  std::vector<float3> shortest_path = mid_map->get_shortest_path(curr_pose);
//  std::cout<<"****"<<shortest_path.size()<<std::endl;
  for (size_t i=0;i<shortest_path.size();i++)
  {
    pcl::PointXYZRGB clrP;
    clrP.x = shortest_path[i].x;
    clrP.y = shortest_path[i].y;
    clrP.z = 0;
    pclOut->points.push_back (clrP);
  }

  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);

  path_pub->publish (pclOut);

  pclOut->clear();

#endif
  float2 diff = make_float2(start_pos.x,start_pos.y) - lines[current_line_id].b;

  if (sqrtf(dot(diff,diff)) < 1.5f)
  {
    current_line_id ++;

    if (current_line_id + 1 >= lines.size())
      current_line_id = 0;

    geometry_msgs::PoseStamped glb_goal;
    glb_goal.pose.position.x = lines[current_line_id].b.x;
    glb_goal.pose.position.y = lines[current_line_id].b.y;
    glb_goal.pose.position.z = 0;

    tf::Quaternion quat;
    quat.setRPY( 0, 0, lines[current_line_id].tht );
    tf::quaternionTFToMsg(quat, glb_goal.pose.orientation);

    glb_goal_pub->publish(glb_goal);
  }

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "Middle planning time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
            << " ms" << std::endl;

  first_run = false;
}
//---
float get_heading(const nav_msgs::Odometry &odom)
{
  double phi,theta,psi;
  tf::Quaternion q(odom.pose.pose.orientation.x,
                   odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z,
                   odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  return psi;
}
//---
void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  curr_pose.x = msg->pose.pose.position.x;
  curr_pose.y = msg->pose.pose.position.y;
  curr_pose.z = get_heading(*msg);
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  pc_pub = new ros::Publisher;
  path_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  glb_goal_pub = new ros::Publisher;
  vis_pub = new ros::Publisher;

  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/lowres_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber slam_odom_sub = nh.subscribe("/UgvOdomTopic", 1, &slam_odo_call_back);
  ros::Subscriber mission_sub = nh.subscribe("/start_mission", 1, &load_mission_callback);

  *pc_pub = nh.advertise<PointCloud> ("/nf1", 1);
  *path_pub = nh.advertise<PointCloud> ("/path", 1);
  *mid_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("/mid_layer/goal",1);
  *glb_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1);
  *vis_pub = nh.advertise<visualization_msgs::Marker>("path_viz",1);

  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,0.0);


  glb_plan_timer = nh.createTimer(ros::Duration(0.5), glb_plan);

  ros::spin();

  delete pc_pub;
  delete nf1_pub;
  delete path_pub;
  delete vis_pub;

  if (mid_map)
  {
    delete mid_map;
    delete a_map;
    delete mid_goal_pub;
    delete glb_goal_pub;
  }
  return 0;
}