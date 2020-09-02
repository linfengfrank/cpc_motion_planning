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
private:
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;
};

#endif // DIJKSTRA_H
