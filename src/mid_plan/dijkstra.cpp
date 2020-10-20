#include "mid_plan/dijkstra.h"
#include <ros/console.h>

Dijkstra::Dijkstra(int maxX, int maxY, int maxZ):
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
void Dijkstra::dijkstra_with_theta(CUDA_GEO::coord glb_tgt)
{
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  // insert the root
  m=m_getNode(mc);
  if (m)
  {
    for (int z=0;z<THETA_GRID_SIZE;z++) {
      mc.z=z;
      m=m_getNode(mc);
      m->g = 0 + mm_obsCostAt(mc,0);
      _PQ.insert(m,m->g);
    }

  }
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;
    for (shift_child child : children[positive_modulo(mc.z,THETA_GRID_SIZE)])
    {
      update_neighbour(mc, child.shift, m, child.cost);
    }
  }
}

void Dijkstra::hybrid_dijkstra_with_int_theta(CUDA_GEO::coord glb_tgt)
{
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  nodeInfo* p;

  // insert the root
  m=m_getNode(mc);
  if (m)
  {
    for (int z=0;z<THETA_GRID_SIZE;z++)
    {
      mc.z=z;
      m=m_getNode(mc);
      m->pose = coord2float3(mc);
      m->ptr2parent = nullptr;
      m->g = 0 + mm_obsCostAt(mc,0);
      _PQ.insert(m,m->g);
    }
  }
  // start the hybrid D
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;

    for (shift_child child : children[positive_modulo(mc.z,THETA_GRID_SIZE)])
    {
      hybrid_update_neighbour(child.shift_pose, m, child.cost);
    }
  }
}

void Dijkstra::hybrid_dijkstra_with_int_theta_with_line(CUDA_GEO::coord glb_tgt,CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b)
{
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  nodeInfo* p;

  // insert the root
  m=m_getNode(mc);
  if (m)
  {
    for (int z=0;z<THETA_GRID_SIZE;z++)
    {
      mc.z=z;
      m=m_getNode(mc);
      m->pose = coord2float3(mc);
      m->ptr2parent = nullptr;
      m->g = 0 + mm_obsCostAt(mc,0);
      _PQ.insert(m,m->g);
    }
  }
  // start the hybrid D
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;

    for (shift_child child : children[positive_modulo(mc.z,THETA_GRID_SIZE)])
    {
      hybrid_update_neighbour_with_line(child.shift_pose, m, child.cost, seg_a, seg_b);
    }
  }
}

void Dijkstra::update_neighbour(const CUDA_GEO::coord &c, const int3 &shift, nodeInfo* m, float trans_cost)
{
  CUDA_GEO::coord pc;
  pc.x = c.x + shift.x;
  pc.y = c.y + shift.y;
  pc.z = c.z + shift.z;

  pc.z = positive_modulo(pc.z,THETA_GRID_SIZE);
  nodeInfo* p = m_getNode(pc);

  if (p)
  {
    if (!p->inClosed)
    {
      float new_g = trans_cost +
          m->g + mm_obsCostAt(pc,0);


      if (p->g > new_g)
      {
        p->g = new_g;
        p->ptr2parent = m;
        _PQ.insert(p,p->g);
      }
    }
  }
}

void Dijkstra::hybrid_update_neighbour(const float3 &shift_pose, nodeInfo *m, float trans_cost)
{
  float3 child_pose = shift_pose + m->pose;
  CUDA_GEO::coord pc = float32coord(child_pose);
  nodeInfo* p = m_getNode(pc);

  if (p && !p->inClosed)
  {
    float new_g = trans_cost + m->g + mm_obsCostAt(pc,0);
    if (p->g > new_g)
    {
      p->g = new_g;
      p->pose = child_pose;
      p->ptr2parent = m;
      _PQ.insert(p,p->g);
    }
  }
}

void Dijkstra::hybrid_update_neighbour_with_line(const float3 &shift_pose, nodeInfo *m, float trans_cost,const CUDA_GEO::pos &seg_a, const CUDA_GEO::pos &seg_b)
{
  float3 child_pose = shift_pose + m->pose;
  CUDA_GEO::coord pc = float32coord(child_pose);
  nodeInfo* p = m_getNode(pc);

  if (p && !p->inClosed)
  {
    float new_g = trans_cost + m->g + mm_obsCostAt(pc,0);

    CUDA_GEO::pos pc_pos = coord2pos(pc);
    pc_pos.z = 0;
    float lat_dist = straight_line_proj(seg_a,seg_b,pc_pos);
    new_g += lat_dist*lat_dist;

    if (p->g > new_g)
    {
      p->g = new_g;
      p->pose = child_pose;
      p->ptr2parent = m;
      _PQ.insert(p,p->g);
    }
  }
}


