#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/grid_graph.h>
#include <queue>

class Dijkstra : public GridGraph
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
  void dijkstra2D_with_line(CUDA_GEO::coord glb_tgt, CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b);
  void bfs2D(CUDA_GEO::coord glb_tgt);
  void dijkstra3D(CUDA_GEO::coord glb_tgt);
  void dijkstra_with_theta(CUDA_GEO::coord glb_tgt);
  void update_neighbour(const CUDA_GEO::coord &c, const int3 &shift, const float &m_g);
  ~Dijkstra()
  {
    delete [] _id_map;
    delete [] _init_id_map;
  }


  float m_getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const
  {
    if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=8)
    {
      return default_value;
    }
    else
    {
      return m_id_map[coord2index(s)].g;
    }
  }

private:
  int3 children[8][4];
  nodeInfo *m_id_map; // Identity map, store Dijkstra related params
  nodeInfo *m_init_id_map; // A copy for reset
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;

  int positive_modulo(int i, int n)
  {
    return (i % n + n) % n;
  }


  nodeInfo* m_getNode(CUDA_GEO::coord s)
  {
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=8)
      return nullptr;

    return &m_id_map[coord2index(s)];
  }

  float m_obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied)
  {
    s.z = 0;
    return obsCostAt(s,default_value,occupied);
  }
};

#endif // DIJKSTRA_H
