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
#include <mid_plan/dijkstra.h>
#include <mid_plan/a_star.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/line_target.h>
#define SHOWPC

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);
ros::Publisher* nf1_pub;
ros::Publisher* vis_pub;
ros::Publisher* pc_pub;
ros::Publisher* mid_goal_pub;
ros::Publisher* line_goal_pub;
Dijkstra *mid_map=nullptr;
Astar *a_map=nullptr;

float FLY_HEIGHT;
bool first_run = true;
CUDA_GEO::pos curr_target_pos;
bool stucked = false;
bool received_cmd = false;
bool received_map = false;
bool received_ref = false;
ros::Timer glb_plan_timer;
cpc_motion_planning::ref_data ref;
cpc_aux_mapping::grid_map nf1_map_msg;

//---
struct line_seg
{
  float2 a;
  float2 b;
  float2 uni;
  float tht;
  float dist;
  bool do_turning;
  line_seg(float2 a_, float2 b_, bool do_turning_):a(a_),b(b_),do_turning(do_turning_)
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
      //for (int z=0;z<msg->z_length;z++)
      {
        c.x = x;
        c.y = y;
        c.z = tgt_height_coord;
        float d_c = mid_map->getCost2Come(c,0.0f)*15;
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
    msg.z_size = 1;
    msg.payload8.resize(sizeof(CostTheta)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map,int tgt_height_coord)
{
  CUDA_GEO::coord c;
  CostTheta *tmp = static_cast<CostTheta*>(static_cast<void*>(msg.payload8.data()));
  int i=0;
  for (int y=0;y<map->getMaxY();y++)
  {
    for (int x=0;x<map->getMaxX();x++)
    {
      //for (int z=0;z<map->getMaxZ();z++)
      {
        c.x = x;
        c.y = y;
        c.z = tgt_height_coord;
        tmp[i].c=mid_map->getCost2Come(c,0.0f);
        tmp[i].t=mid_map->getTheta(c,0.0f);
        i++;
      }
    }
  }
}
//---
void mapCallback(const cpc_aux_mapping::grid_map::ConstPtr& msg)
{
  received_map = true;
  CUDA_GEO::pos origin;
  if (mid_map == nullptr)
  {
    mid_map = new Dijkstra(msg->x_size,msg->y_size,msg->z_size);
    setup_map_msg(nf1_map_msg,mid_map,true);

    a_map = new Astar(msg->x_size,msg->y_size,msg->z_size);
  }
  mid_map->copyEdtData(msg);
  a_map->copyEdtData(msg);
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
void send_line_target(int idx)
{
  cpc_motion_planning::line_target line_tgt;
  line_tgt.target_pose.pose.position.x = lines[idx].b.x;
  line_tgt.target_pose.pose.position.y = lines[idx].b.y;
  line_tgt.target_pose.pose.position.z = 0;

  tf::Quaternion quat;
  // turn towards the next line
  if (idx + 1 < lines.size())
    quat.setRPY( 0, 0, lines[idx+1].tht );
  else
    quat.setRPY( 0, 0, lines[idx].tht );
  tf::quaternionTFToMsg(quat, line_tgt.target_pose.pose.orientation);

  line_tgt.do_turning = lines[idx].do_turning;
  line_tgt.reaching_radius = 1.0f;

  line_goal_pub->publish(line_tgt);
}
//---
void load_mission_callback(const std_msgs::Int32::ConstPtr &msg)
{
  // Read in the data files
  std::ifstream corridor_file;
  float data[3];
  std::vector<float2> wps;
  std::vector<bool> turning_list;
  corridor_file.open("/home/ugv/yzchen_ws/omen_deploy/in.txt");

  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1]>>data[2])
    {
      wps.push_back((make_float2(data[0],data[1])));
      turning_list.push_back(data[2]>0);
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
      lines.push_back(line_seg(wps[i],wps[i+1],turning_list[i+1]));
    }
    received_cmd = true;
    current_line_id = 0;
    send_line_target(current_line_id);
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
    glb_tgt = mid_map->rayCast(mid_map->pos2coord(proj_pnt),glb_tgt).back();
  }
  else
  {
    glb_tgt = mid_map->rayCast(start,mid_map->pos2coord(proj_pnt)).back();
    use_line = false;
  }

  //---

  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = a_map->AStar2D(glb_tgt,start,false,length);

//  if (!use_line)
//    mid_map->dijkstra2D(path[0]);
//  else
    mid_map->dijkstra2D_with_line(path[0],seg_a,seg_b);

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
#endif

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Middle planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms" << std::endl;

  first_run = false;
}
//---
void target_reached_callback(const std_msgs::Int32::ConstPtr &msg)
{
  if (current_line_id + 1 <lines.size())
  {
    current_line_id ++;
    send_line_target(current_line_id);
  }
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");
  pc_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  vis_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  line_goal_pub = new ros::Publisher;
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber mission_sub = nh.subscribe("/start_mission", 1, &load_mission_callback);
  ros::Subscriber target_reached_sub = nh.subscribe("/target_reached", 1, &target_reached_callback);
  *pc_pub = nh.advertise<PointCloud> ("/nf1", 1);
  *mid_goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mid_goal", 1);
  *line_goal_pub = nh.advertise<cpc_motion_planning::line_target> ("/line_target", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("/mid_layer/goal",1);
  *vis_pub = nh.advertise<visualization_msgs::Marker>("path_viz",1);

  pclOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,0.0);


  glb_plan_timer = nh.createTimer(ros::Duration(0.333), glb_plan);

  ros::spin();

  delete pc_pub;
  delete nf1_pub;
  delete vis_pub;

  if (mid_map)
  {
    delete mid_map;
    delete a_map;
    delete mid_goal_pub;
    delete line_goal_pub;
  }
  return 0;
}
