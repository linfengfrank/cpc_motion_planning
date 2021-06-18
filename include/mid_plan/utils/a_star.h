#ifndef A_STAR_H
#define A_STAR_H

#include <mid_plan/utils/grid_graph.h>
#include <queue>

class Astar : public GridGraph
{
public:
  Astar(int maxX, int maxY, int maxZ);
  std::vector<CUDA_GEO::coord> AStar2D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone, float &length, float safety_radius);

  std::vector<CUDA_GEO::coord> AStar3D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone, float &length,
                                       const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_val_map = nullptr);

  unsigned int findTargetCoord(const std::vector<CUDA_GEO::coord> &path);
  unsigned int findTargetCoordLos(std::vector<CUDA_GEO::coord> path, CUDA_GEO::coord start, unsigned int start_idx, float look_ahead_diff=0.5f);
  std::vector<CUDA_GEO::pos> findSplitCoords(const std::vector<CUDA_GEO::coord> &path);
  bool checkTopo(const std::vector<CUDA_GEO::coord> &path_a,const std::vector<CUDA_GEO::coord> &path_b);

  inline float point2lineDist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2, const CUDA_GEO::coord & c0, float scale_verticle = 1.0f)
  {
    CUDA_GEO::coord a = c1-c0;
    CUDA_GEO::coord b = c2-c1;

    a.z = static_cast<int>(a.z * scale_verticle);
    b.z = static_cast<int>(b.z * scale_verticle);

    int a_dot_b = a.x*b.x + a.y*b.y + a.z*b.z;

    if (b.square() == 0)
      return sqrtf(static_cast<float>(a.square()));

    return sqrtf(static_cast<float>(a.square()*b.square() - a_dot_b*a_dot_b)/static_cast<float>(b.square()));
  }

  CUDA_GEO::coord get_first_free_coord(CUDA_GEO::coord start,float safety_radius)
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

  std::queue<nodeInfo*> _Q;

};

#endif // A_STAR_H
