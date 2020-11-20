#include "glb_plan/global_planner.h"

GlobalPlanner::GlobalPlanner()
{

}

bool GlobalPlanner::load_c_map(std::string filename)
{
  bool ok = m_c_map.Load(filename.c_str());

  m_origin = CUDA_GEO::pos(-30,-30,0);
  m_step_width = 0.05f;
  m_width = 1000;
  m_height = 1000;

  m_a_map = new Astar(m_width,m_height,1);
  m_a_map->setMapSpecs(m_origin,m_step_width);

  m_c = new int[m_width*m_height];
  m_g = new int[m_width*m_height];
  m_h = new int[m_width*m_height];
  m_s = new int[m_width*m_height];
  m_t = new int[m_width*m_height];

  build_axis_aligned_map();
  return ok;
}

// This is currently the 2D version
void GlobalPlanner::perform_edt()
{
  for (int x=0;x<m_width; x++)
  {
    phase1(x);
  }

  for (int y=0;y<m_height; y++)
  {
    phase2(y);
  }

  for (int x=0; x<m_width; x++)
  {
    for (int y=0; y<m_height; y++)
    {
      m_a_map->getEdtMapPtr()[toid(x,y)].d = sqrtf(m_h[toid(x,y)]);
    }
  }
}




void GlobalPlanner::phase1(int x)
{
  // scan 1
  if (m_c[toid(x,0)] > 100)
    m_g[toid(x,0)]=0;
  else
    m_g[toid(x,0)] = 10000;

  for (int y=1; y<m_height; y++)
  {
    if (m_c[toid(x,y)] > 100)
      m_g[toid(x,y)]=0;
    else
      m_g[toid(x,y)] = 1 + m_g[toid(x,y-1)];
  }
  // scan 2
  for (int y=m_height-2; y>=0; y--)
  {
    if (m_g[toid(x,y+1)] < m_g[toid(x,y)])
    {
      m_g[toid(x,y)] = 1+m_g[toid(x,y+1)];
    }
  }
}

void GlobalPlanner::phase2(int y)
{
  int q=0;
  m_s[toid(0,y)] = 0;
  m_t[toid(0,y)] = 0;
  for (int u=1;u<m_width;u++)
  {
    while (q>=0 && f(m_t[toid(q,y)],m_s[toid(q,y)],y)
           > f(m_t[toid(q,y)],u,y))
    {
      q--;
    }
    if (q<0)
    {
      q = 0;
      m_s[toid(0,y)]=u;
    }
    else
    {
      int w = 1+sep(m_s[toid(q,y)],u,y);
      if (w < m_width)
      {
        q++;
        m_s[toid(q,y)]=u;
        m_t[toid(q,y)]=w;
      }
    }
  }
  for (int u=m_width-1;u>=0;u--)
  {
    m_h[toid(u,y)]=f(u,m_s[toid(q,y)],y);
    if (u == m_t[toid(q,y)])
      q--;
  }
}

std::vector<CUDA_GEO::pos> GlobalPlanner::plan(const CUDA_GEO::pos &goal_pos, const CUDA_GEO::pos &start_pos)
{
  std::vector<CUDA_GEO::pos> output;
  CUDA_GEO::coord start = m_a_map->pos2coord(start_pos);
  CUDA_GEO::coord goal = m_a_map->pos2coord(goal_pos);
  float length = 0.0f;
  std::vector<CUDA_GEO::coord> coord_path =  m_a_map->AStar2D(goal,start,false,length);
  std::reverse(coord_path.begin(),coord_path.end());

  for (size_t i=0; i<coord_path.size();i++)
  {
    output.push_back(m_a_map->coord2pos(coord_path[i]));
  }

  return output;
}
