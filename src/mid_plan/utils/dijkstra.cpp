#include "mid_plan/utils/dijkstra.h"
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

void Dijkstra::dijkstra2D_with_line_map(CUDA_GEO::coord glb_tgt, EDTMap* line_map, bool is_path_blocked, float safety_radius)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = glb_tgt;
  CUDA_GEO::coord pc;
  CUDA_GEO::pos pc_pos;
  float line_dist;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  if (m)
  {
    m->g = 0 + obsCostAt(mc,0,safety_radius);
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
                m->g + obsCostAt(pc,0,safety_radius);

            if (!is_path_blocked)
            {
              pc_pos = coord2pos(pc);
              line_dist = line_map->getEDT(make_float2(pc_pos.x,pc_pos.y));
              new_g += 0.01f*line_dist*line_dist;
            }

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
            new_g += 0.5f*lat_dist*lat_dist;

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

    obsCostAt(mc,0,occupied,false,safety_radius);
    if (!occupied)
    {
      found_free = true;
      break;
    }

    for (int ix=-1;ix<=1;ix++)
    {
      for (int iy=-1;iy<=1;iy++)
      {
        if ((ix==0 && iy ==0))
          continue;

        pc.x = mc.x + ix;
        pc.y = mc.y + iy;
        pc.z = mc.z;
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

// Given a coord mc, find the id of its EDT root from the line_map
bool Dijkstra::get_root_idx(int &idx, const CUDA_GEO::coord &mc, EDTMap* line_map)
{
  // Transform the corresponding coordinate from the dijkstra map to the line_map
  CUDA_GEO::pos tmp = coord2pos(mc);
  CUDA_GEO::coord tmp_c = line_map->pos2coord(tmp);

  if (line_map->isInside(tmp_c))
  {
    idx = line_map->m_hst_sd_map[line_map->to_id(tmp_c.x,tmp_c.y,0)].rt_idx;
    return true;
  }
  else
  {
    idx = -1;
    return false;
  }
}

void Dijkstra::update_selected_tgt(CUDA_GEO::coord& sel_tgt, float &min_h, int& max_path_id, const CUDA_GEO::coord &mc, const CUDA_GEO::coord &goal, float safety_radius,
                                   EDTMap* line_map, const std::unordered_map<int, pathPntInfo> *lpta)
{
  float d2goal = dist(mc,goal);
  CUDA_GEO::pos m_pos = coord2pos(mc);
  float d2line = line_map->getEDT(make_float2(m_pos.x,m_pos.y));

  if(d2line >= line_map->m_grid_step*10)
    return; // too far from the nominal line

  if (lpta != nullptr)
  {
    int idx;
    if (!get_root_idx(idx, mc, line_map))
      return; // mc not in the line_map

    if (lpta->find(idx) == lpta->end())
      return; // cannot find the idx in lpta

    // get the id of the root along the path
    int path_id = lpta->at(idx).path_idx;

    // if path_id is large (closer to finishing line)
    // or if the distance to goal is closer
    if (path_id > max_path_id || (path_id == max_path_id && min_h > d2goal))
    {
      // collision checking
      float target_angle = lpta->at(idx).desired_angle;
      CUDA_GEO::pos c_r,c_f;
      calculate_bounding_centres(coord2pos(mc),target_angle,c_r,c_f);
      if(!two_circle_collision(c_r,c_f,safety_radius))
      {
        max_path_id = path_id;
        min_h = d2goal;
        sel_tgt = mc;
      }
    }
  }
  else
  {
    if(min_h > d2goal)
    {
      min_h = d2goal;
      sel_tgt = mc;
    }
  }
}

CUDA_GEO::coord Dijkstra::find_available_target_with_line(CUDA_GEO::coord start, CUDA_GEO::coord goal, EDTMap* line_map, float safety_radius, const std::unordered_map<int, pathPntInfo> *lpta)
{
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
  float min_h = std::numeric_limits<float>::infinity();
  //Get the local Dijkstra target
  CUDA_GEO::coord mc = start;
  CUDA_GEO::coord pc, selected_tgt;
  nodeInfo* m;
  // insert the root
  m=getNode(mc);

  int max_path_id = -1;
  bool occupied=false;
  if (m)
  {
    _Q.push(m);
    update_selected_tgt(selected_tgt,min_h,max_path_id,m->c,goal,safety_radius,line_map,lpta);
  }
  while (_Q.size()>0)
  {
    m=_Q.front();
    _Q.pop();
    m->inClosed = true;
    mc = m->c;

    update_selected_tgt(selected_tgt,min_h,max_path_id,mc,goal,safety_radius,line_map,lpta);

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

        if (p && !p->inClosed)
        {
          obsCostAt(pc,0,occupied,false,safety_radius);
          if (!occupied)
          {
            p->inClosed = true;
            _Q.push(p);
          }
        }
      }
    }
  }

  std::cout<<"max_path_id: "<<max_path_id<<std::endl;

  if (std::isinf(min_h))
    selected_tgt = start;

  return selected_tgt;
}


