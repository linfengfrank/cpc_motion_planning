#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H
#include "glb_plan/map.h"
#include "mid_plan/utils/a_star.h"
#include <string>

class GlobalPlanner
{
public:
  GlobalPlanner();
  bool load_c_map(std::string filename);
  void perform_edt();
  std::vector<CUDA_GEO::pos> plan(const CUDA_GEO::pos &goal, const CUDA_GEO::pos &start);

public:
  void phase1(int x);
  void phase2(int y);
  int toid(int x, int y)
  {
    return y*m_width + x;
  }

  inline int f(int x,int i,int y)
  {
      return (x-i)*(x-i) + m_g[toid(i,y)]*m_g[toid(i,y)];
  }

  inline int sep(int i,int u,int y)
  {
      return (u*u-i*i+ m_g[toid(u,y)]*m_g[toid(u,y)] - m_g[toid(i,y)]*m_g[toid(i,y)])/(2*(u-i));
  }

  void build_axis_aligned_map()
  {
    CUDA_GEO::coord c;
    CUDA_GEO::pos p;
    POINT pnt;
    for(c.x=0; c.x<m_width; c.x++)
    {
      for (c.y=0; c.y<m_height; c.y++)
      {
        p = m_a_map->coord2pos(c);
        m_c[toid(c.x,c.y)] = m_c_map.Grey(p.x,p.y);
      }
    }
  }



public:
  CUDA_GEO::pos m_origin;
  float m_step_width;
  int m_width;
  int m_height;
  CMap m_c_map;
  Astar *m_a_map;
  int *m_c;
  int *m_g;
  int *m_h;
  int *m_s;
  int *m_t;
};

#endif // GLOBAL_PLANNER_H
