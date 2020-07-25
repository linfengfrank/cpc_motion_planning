#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/grid_graph.h>
#include <queue>

class Dijkstra : public GridGraph
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
  void bfs2D(CUDA_GEO::coord glb_tgt);
  void dijkstra3D(CUDA_GEO::coord glb_tgt);
private:
  std::queue<nodeInfo*> _Q;
  std::queue<nodeInfo*> _OQ;
};

#endif // DIJKSTRA_H
