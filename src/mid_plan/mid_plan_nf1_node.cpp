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


#include <ubfs/cuda_mid_map.cuh>
#include <ubfs/bfs_wrapper.cuh>
#define SHOWPC

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr pclOut (new PointCloud);

typedef pcl::PointCloud<pcl::PointXYZ> ObsCloud;
ObsCloud::Ptr obsOut (new ObsCloud);

ros::Publisher* nf1_pub;
ros::Publisher* pc_pub;
ros::Publisher* obs_pub;
ros::Publisher* mid_goal_pub;
Dijkstra *mid_map=nullptr;
Astar *a_map=nullptr;

NF1Map3D *nf1map3d =nullptr;
float *costPc = nullptr;
ubfs_cls *ubfssb =nullptr;

float FLY_HEIGHT;
bool first_run = true;
CUDA_GEO::pos curr_target_pos;
bool stucked = false;
bool received_cmd = false;
bool received_map = false;
bool received_ref = false;
CUDA_GEO::pos goal;
ros::Timer glb_plan_timer;
ros::Timer profile_timer;
cpc_motion_planning::ref_data ref;
cpc_aux_mapping::grid_map nf1_map_msg;

void pub_cudaMap(int tgt_height_coord)
{
  nf1map3d->cost_d2h();
  CUDA_GEO::coord c;
  CUDA_GEO::pos p;
  for ( c.x=0;c.x<nf1map3d->m_map_size.x;c.x++)
  {
    for ( c.y=0;c.y<nf1map3d->m_map_size.y;c.y++)
    {
//      for ( c.z=0;c.z<nf1map3d->m_map_size.z;c.z++)
//      {
        c.z=tgt_height_coord;

        int idx = c.z*nf1map3d->m_map_size.x*nf1map3d->m_map_size.y+
            c.y*nf1map3d->m_map_size.x+c.x;

        if(nf1map3d->h_cost_to_go[idx]>=FLT_MAX-1)
          continue;
        float val = nf1map3d->h_cost_to_go[idx] *15;

        int d = static_cast<int>(val);
        if (d==0)
        {
          continue;
        }
        if(d>255)
          d = 255;
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

//      }
    }
  }

  pcl_conversions::toPCL(ros::Time::now(), pclOut->header.stamp);

  pc_pub->publish (pclOut);
  pclOut->clear();

//  // publish obs bdr
//  nf1map3d->obs_dense_d2h();
//  for (int i=0;i<nf1map3d->obs_num;i++)
//  {
//    int3 obs_crd = nf1map3d->obs_dense_h[i];
//    CUDA_GEO::coord co(obs_crd.x,obs_crd.y,obs_crd.z);
//    p=mid_map->coord2pos(co);
//    pcl::PointXYZ obsP;
//    obsP.x= p.x;
//    obsP.y = p.y;
//    obsP.z = p.z;

//    obsOut->points.push_back(obsP);
//  }
//  pcl_conversions::toPCL(ros::Time::now(), obsOut->header.stamp);
//  obs_pub->publish( obsOut);
//  printf("obs cloud size is %d \n",obsOut->size());
//  obsOut->clear();
}
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
    msg.z_size = map->getMaxZ();  //1
//    printf("max z is %d\n",msg.z_size);
    msg.payload8.resize(sizeof(float)*static_cast<unsigned int>(msg.x_size*msg.y_size*msg.z_size));
  }

  msg.type = cpc_aux_mapping::grid_map::TYPE_NF1;
}
//---
void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map,int tgt_height_coord)
{
  CUDA_GEO::coord c;
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
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
        float d_c = mid_map->getCost2Come(c,0.0);
        tmp[i++]=d_c;
      }
    }
  }
}
//---
void nf1map2msg(cpc_aux_mapping::grid_map &msg)
{
  float *tmp = static_cast<float*>(static_cast<void*>(msg.payload8.data()));
  CUDA_MEMCPY_D2H(tmp,nf1map3d->d_cost_to_go,nf1map3d->m_byte_size);

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


  // copy seendist msg to cuda

  CUDA_MEMCPY_H2D(nf1map3d->d_val_map,msg->payload8.data(),nf1map3d->m_edt_size);
}
//---
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  received_cmd = true;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = msg->pose.position.z;
}

void profile_sssp(const ros::TimerEvent&)
{

  int3 tgt_i3{50,50,50};
 auto start_time = std::chrono::steady_clock::now();
  ubfssb->ubfs_sssp3d_wrapper(tgt_i3,nf1map3d);
    auto end_time = std::chrono::steady_clock::now();
        std::cout << "Middle planning time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                  << " ms" << std::endl;
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

  CUDA_GEO::coord glb_tgt = mid_map->pos2coord(goal);
  glb_tgt.z = tgt_height_coord;
  glb_tgt = mid_map->rayCast(start,glb_tgt).back();

  float length = 0.0f;
  std::vector<CUDA_GEO::coord> path = a_map->AStar2D(glb_tgt,start,false,length);

  //  mid_map->bfs2D(path[0]);

//  setup_map_msg(nf1_map_msg,mid_map,false);
  //  copy_map_to_msg(nf1_map_msg,mid_map,tgt_height_coord);

  // cuda bfs


  int3 tgt_i3{path[0].x,path[0].y,path[0].z};
  ubfssb->ubfs_sssp3d_wrapper(tgt_i3,nf1map3d);
  // setupmsg: z not 1
  setup_map_msg(nf1_map_msg,mid_map,false);
  nf1map2msg(nf1_map_msg);
  nf1_pub->publish(nf1_map_msg);
#ifdef SHOWPC
//  publishMap(tgt_height_coord);
  pub_cudaMap(tgt_height_coord);
#endif

  auto end_time = std::chrono::steady_clock::now();
  //      std::cout << "Middle planning time: "
  //                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
  //                << " ms" << std::endl;

  first_run = false;
}
//---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_layer_node");

  int3 map_size = int3{100,100,100};
  nf1map3d =new NF1Map3D(map_size,0.2);
  nf1map3d->setup_device();
  ubfssb = new ubfs_cls(0,nf1map3d->m_volume);

  pc_pub = new ros::Publisher;
  obs_pub = new ros::Publisher;
  nf1_pub = new ros::Publisher;
  mid_goal_pub = new ros::Publisher;
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/edt_map", 1, &mapCallback);
  ros::Subscriber stuck_sub = nh.subscribe("/stuck", 1, &stuckCallback);
  ros::Subscriber glb_tgt_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);

  *pc_pub = nh.advertise<PointCloud> ("/nf1", 1);
  *obs_pub = nh.advertise<PointCloud>("/obs_bdr",1);
  *mid_goal_pub = nh.advertise<PointCloud> ("/mid_goal", 1);
  *nf1_pub = nh.advertise<cpc_aux_mapping::grid_map>("/mid_layer/goal",1);

  pclOut->header.frame_id = "/world";
  obsOut->header.frame_id = "/world";
  nh.param<float>("/nndp_cpp/fly_height",FLY_HEIGHT,2.0);


  glb_plan_timer = nh.createTimer(ros::Duration(1), glb_plan);
  profile_timer = nh.createTimer(ros::Duration(1), profile_sssp);

  ros::spin();

  delete pc_pub;
  delete obs_pub;
  delete nf1_pub;

  if (mid_map)
  {
    delete mid_map;
    delete a_map;
  }
  nf1map3d->free_device();

  return 0;
}
