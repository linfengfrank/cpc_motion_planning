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

  float dt[4]={0.25f,0.6f,1.1f,2.1f};
  for (size_t i = 0; i< THETA_GRID_SIZE; i++)
  {
    float theta = grid2theta(static_cast<int>(i));
    for (int wg = -1; wg<=1;wg++)
    {
      for (int vg = -1; vg<=1;vg++)
      {
        if (wg == 0 && vg == 0)
          continue;

        float w = 0.25f*static_cast<float>(wg);
        float v = 0.6f*static_cast<float>(vg);
        for (size_t k=0; k<4; k++)
        {
          shift_child child = get_shift_child(theta,w,v,dt[k]);
          if (child.shift.x == 0 && child.shift.y == 0 && child.shift.z == 0)
            continue;

          children[i].insert(child);
        }
      }
    }
  }
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
                m->g + obsCostAt(pc,0,occupied,true);

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
                m->g + obsCostAt(pc,0,occupied,true);

            pc_pos = coord2pos(pc);
            lat_dist = straight_line_proj(seg_a,seg_b,pc_pos);
            new_g += lat_dist*lat_dist;

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

void Dijkstra::dijkstra_with_theta(CUDA_GEO::coord glb_tgt)
{
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  nodeInfo* m;
  // insert the root
  m=m_getNode(mc);

//  bool occupied=false;
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
      update_neighbour(mc, child.shift, m->g, child.cost);
    }

    //children[positive_modulo(mc.z,THETA_GRID_SIZE)]
//    for (size_t i = 0; i < 4; i++)
//    {
//      update_neighbour(mc, children[positive_modulo(mc.z,THETA_GRID_SIZE)][i], m->g);
//    }


    // get all neighbours
//    for (int ix=-1;ix<=1;ix++)
//    {
//      for (int iy=-1;iy<=1;iy++)
//      {
//        for (int iz=-1;iz<=1;iz++)
//        {
//          if (ix==0 && iy ==0 && iz == 0)
//            continue;

//          pc.x = mc.x + ix;
//          pc.y = mc.y + iy;
//          pc.z = mc.z + iz;
//          nodeInfo* p = getNode(pc);
//          if (p)
//          {
//            if (!p->inClosed)
//            {
//              float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy+iz*iz))*getGridStep() +
//                  m->g + obsCostAt(pc,0,occupied);
//              if (p->g > new_g)
//              {
//                p->g = new_g;
//                _PQ.insert(p,p->g);
//              }
//            }
//          }
//        }
//      }
//    }
  }
}

void Dijkstra::update_neighbour(const CUDA_GEO::coord &c, const int3 &shift, const float &m_g, float trans_cost)
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
//      float new_g = sqrtf(static_cast<float>(shift.x*shift.x+shift.y*shift.y))*getGridStep() +
//          fabsf(static_cast<float>(shift.z))*12*getGridStep() +
//          m_g + m_obsCostAt(pc,0,occupied);
      float new_g = trans_cost +
          m_g + mm_obsCostAt(pc,0);


      if (p->g > new_g)
      {
        p->g = new_g;
        _PQ.insert(p,p->g);
      }
    }
  }
}


