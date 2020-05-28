#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/DijkstraMap.h>

class Dijkstra : public DijkstraMap
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
};

#endif // DIJKSTRA_H
