#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/utils/grid_graph.h>
#include <queue>

class Dijkstra : public GridGraph
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
  void dijkstra2D_with_line_map(CUDA_GEO::coord glb_tgt, EDTMap* line_map, bool is_path_blocked);
  void dijkstra2D_with_line(CUDA_GEO::coord glb_tgt, CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b);
  CUDA_GEO::coord find_available_target_with_line(CUDA_GEO::coord start, CUDA_GEO::coord goal, EDTMap* line_map);
  void dijkstra3D(CUDA_GEO::coord glb_tgt);
  CUDA_GEO::coord get_first_free_coord(CUDA_GEO::coord start);
  void update_selected_tgt(CUDA_GEO::coord& sel_tgt, float &min_h, const CUDA_GEO::coord &mc, const CUDA_GEO::coord &goal, EDTMap* line_map);
private:
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;
};

#endif // DIJKSTRA_H
