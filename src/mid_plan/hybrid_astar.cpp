#include "mid_plan/hybrid_astar.h"
#include <ros/console.h>

HybridAstar::HybridAstar(int maxX, int maxY, int maxZ):
  GridGraph(maxX,maxY,maxZ)
{
  m_id_map = new nodeInfo[_w*_h*THETA_GRID_SIZE];
  m_init_id_map = new nodeInfo[_w*_h*THETA_GRID_SIZE];

  //Assign coords value for all members in the id map
  CUDA_GEO::coord s;
  for (s.x=0; s.x<_w; s.x++)
  {
    for (s.y=0; s.y<_h; s.y++)
    {
      for (s.z=0; s.z<THETA_GRID_SIZE; s.z++)
      {
        m_init_id_map[coord2index(s)].c=s;
      }
    }
  }
}

std::vector<float3> HybridAstar::plan(const float3 &start_pose, CUDA_GEO::coord glb_tgt)
{
  std::vector<float3> path;

  //First clear the old information
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  _PQ.clear();

  // Useful variables
  CUDA_GEO::coord closest_coord,start;
  float min_h = std::numeric_limits<float>::infinity();

  //Set the root
  start = float32coord(start_pose);
  nodeInfo* ni=m_getNode(start);
  if (ni)
  {
    ni->g = 0;
    ni->h = dist2D(ni->c,glb_tgt);
    ni->pose = start_pose;
    _PQ.insert(ni,ni->g);
    closest_coord = ni->c;
    min_h = ni->h;
  }
  else
  {
    ROS_ERROR("First point outside the boundary.");
    exit(-1);
  }

  // start the hybrid D
  bool reached_free_zone = false;
  while (_PQ.size()>0)
  {
    ni=_PQ.pop();
    ni->inClosed = true;

    if(hybrid_isfree(ni->c))
      reached_free_zone = true;

    if (min_h > ni->h)
    {
      min_h = ni->h;
      closest_coord = ni->c;
    }

    for (shift_child child : children[positive_modulo(ni->c.z,THETA_GRID_SIZE)])
    {

      float3 child_pose = child.shift_pose + ni->pose;
      CUDA_GEO::coord pc = float32coord(child_pose);
      nodeInfo* p = m_getNode(pc);

      if (p && !p->inClosed)
      {
        float new_g = child.cost + ni->g + hybrid_obsCostAt(pc,0);
        if (p->g > new_g && !(!hybrid_isfree(pc) && reached_free_zone))
        {
          p->g = new_g;
          p->pose = child_pose;
          p->ptr2parent = ni;
          p->h = dist2D(p->c,glb_tgt);
          _PQ.insert(p,p->g);
        }
      }
    }
  }

  ni=m_getNode(closest_coord);
  while (ni != nullptr)
  {
    path.push_back(coord2float3(ni->c));
    ni = ni->ptr2parent;
  }

  return path;
}

