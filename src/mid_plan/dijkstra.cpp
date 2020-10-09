#include "mid_plan/dijkstra.h"
#include <ros/console.h>

Dijkstra::Dijkstra(int maxX, int maxY, int maxZ):
  GridGraph(maxX,maxY,maxZ)
{

}

void Dijkstra::dijkstra2D(CUDA_GEO::coord glb_tgt)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  if (m)
  {
    m->g = 0 + obsCostAt(mc,0);
    _PQ.insert(m,m->g);
  }
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;

    // get all neighbours
    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        if (ix==0 && iy ==0)
          continue;

        pc.x = mc.x + ix;
        pc.y = mc.y + iy;
        pc.z = mc.z;
        nodeInfo* p = getNode(pc);
        if (p)
        {
          if (!p->inClosed)
          {
            float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy))*getGridStep() +
                m->g + obsCostAt(pc,0);

            if (p->g > new_g)
            {
              p->g = new_g;
              _PQ.insert(p,p->g);
              p->theta = atan2f(-iy,-ix);
            }
          }
        }
      }
    }
  }
}

void Dijkstra::dijkstra2D_with_line(CUDA_GEO::coord glb_tgt, CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  CUDA_GEO::pos pc_pos;
  float lat_dist;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  if (m)
  {
    m->g = 0 + obsCostAt(mc,0);
    _PQ.insert(m,m->g);
  }
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;

    // get all neighbours
    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        if (ix==0 && iy ==0)
          continue;

        pc.x = mc.x + ix;
        pc.y = mc.y + iy;
        pc.z = mc.z;
        nodeInfo* p = getNode(pc);
        if (p)
        {
          if (!p->inClosed)
          {
            float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy))*getGridStep() +
                m->g + obsCostAt(pc,0);

            pc_pos = coord2pos(pc);
            lat_dist = straight_line_proj(seg_a,seg_b,pc_pos);
            new_g += 0.2f*lat_dist*lat_dist;

            if (p->g > new_g)
            {
              p->g = new_g;
              _PQ.insert(p,p->g);
              p->theta = atan2f(-iy,-ix);
            }
          }
        }
      }
    }
  }
}

void Dijkstra::dijkstra3D(CUDA_GEO::coord glb_tgt)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  bool occupied=false;
  if (m)
  {
    m->g = 0 + obsCostAt(mc,0,occupied);
    _PQ.insert(m,m->g);
  }
  while (_PQ.size()>0)
  {
    m=_PQ.pop();
    m->inClosed = true;
    mc = m->c;

    // get all neighbours
    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        for (int iz=-1;iz<=1;iz++)
        {
          if (ix==0 && iy ==0 && iz == 0)
            continue;

          pc.x = mc.x + ix;
          pc.y = mc.y + iy;
          pc.z = mc.z + iz;
          nodeInfo* p = getNode(pc);
          if (p)
          {
            if (!p->inClosed)
            {
              float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy+iz*iz))*getGridStep() +
                  m->g + obsCostAt(pc,0,occupied);
              if (p->g > new_g)
              {
                p->g = new_g;
                _PQ.insert(p,p->g);
              }
            }
          }
        }
      }
    }
  }
}

void Dijkstra::bfs2D(CUDA_GEO::coord glb_tgt)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  bool occupied=false;
  if (m)
  {
    m->g = 0 + obsCostAt(mc,0,occupied);
    _Q.push(m);
  }
  while (_Q.size()>0)
  {
    m=_Q.front();
    _Q.pop();
    m->inClosed = true;
    mc = m->c;

    // get all neighbours
    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        if ((ix==0 && iy ==0) || ix*iy != 0)
          continue;

        pc.x = mc.x + ix;
        pc.y = mc.y + iy;
        pc.z = mc.z;
        nodeInfo* p = getNode(pc);

        if (p && !p->inClosed)
        {
          obsCostAt(pc,0,occupied);
          p->inClosed = true;
          p->g = 1*getGridStep() + m->g;
          if (!occupied)
          {
            _Q.push(p);
          }
          else
          {
            p->h = 0; // Here h is the distance to obstacle boundary
            _OQ.insert(p,p->h);
          }
        }
      }
    }
  }

  //---------------------
  while (_OQ.size()>0)
  {
    m=_OQ.pop();
    m->inClosed = true;
    mc = m->c;

    // get all neighbours
    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        if ((ix==0 && iy ==0) || ix*iy != 0)
          continue;

        pc.x = mc.x + ix;
        pc.y = mc.y + iy;
        pc.z = mc.z;
        nodeInfo* p = getNode(pc);

        if (p && !p->inClosed)
        {
          float d_dir = sqrtf(static_cast<float>(ix*ix+iy*iy));
          float new_h = d_dir*getGridStep() + m->h;
          if (p->h > new_h)
          {
            p->h = new_h; // Here h is the distance to obstacle boundary
            p->g = d_dir*getGridStep() + m->g;
            _OQ.insert(p,p->h);
          }
        }
      }
    }
  }
}


