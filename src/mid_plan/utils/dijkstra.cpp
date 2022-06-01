#include "mid_plan/dijkstra.h"
#include <ros/console.h>

Dijkstra::Dijkstra(int maxX, int maxY, int maxZ):
  GridGraph(maxX,maxY,maxZ)
{

}

void Dijkstra::dijkstra2D(CUDA_GEO::coord glb_tgt, float obstacle_dist)
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
    m->g = 0 + getObsCostAndCollision(mc,0,occupied,obstacle_dist);
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
                m->g + getObsCostAndCollision(pc,0,occupied,obstacle_dist);
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

void Dijkstra::dijkstra3D(CUDA_GEO::coord glb_tgt, float obstacle_dist)
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
    m->g = 0 + getObsCostAndCollision(mc,0,occupied,obstacle_dist);
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
                  m->g + getObsCostAndCollision(pc,0,occupied,obstacle_dist);
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

void Dijkstra::bfs2D(CUDA_GEO::coord glb_tgt, float obstacle_dist)
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
    m->g = 0 + getObsCostAndCollision(mc,0,occupied,obstacle_dist);
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
          getObsCostAndCollision(pc,0,occupied,obstacle_dist);
          p->inClosed = true;
          p->g = 1*getGridStep() + m->g;
          if (!occupied)
            _Q.push(p);
          else
            _OQ.push(p);
        }
      }
    }
  }

  //---------------------
  while (_OQ.size()>0)
  {
    m=_OQ.front();
    _OQ.pop();
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
          p->inClosed = true;
          p->g = 1*getGridStep() + m->g;
          _OQ.push(p);
        }
      }
    }
  }
}

CUDA_GEO::coord Dijkstra::get_first_free_coord(CUDA_GEO::coord start,float safety_radius)
{
    memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
    CUDA_GEO::coord mc = start;
    CUDA_GEO::coord pc;
    nodeInfo* m;
    m = getNode(mc);

    if (m)
    {
        m->inClosed = true;
        _Q.push(m);
    }

    bool occupied;
    bool found_free=false;
    while (_Q.size()>0)
    {
        m=_Q.front();
        _Q.pop();
        mc = m->c;
        m->inClosed = true;

        getObsCostAndCollision(mc,0,occupied,safety_radius);
        if (!occupied)
        {
            found_free = true;
            break;
        }

        for (int ix=-1;ix<=1;ix++)
        {
            for (int iy=-1;iy<=1;iy++)
            {
                for (int iz=-1;iz<=1;iz++)
                {
                    if ((ix==0 && iy ==0 && iz ==0))
                        continue;

                    pc.x = mc.x + ix;
                    pc.y = mc.y + iy;
                    pc.z = mc.z + iz;
                    nodeInfo* p = getNode(pc);

                    if (p && !p->inClosed)
                    {
                        p->inClosed = true;
                        _Q.push(p);
                    }
                }
            }
        }

        while(!_Q.empty()) _Q.pop();

        if(found_free)
            return mc;
        else
            return start;
    }
}


