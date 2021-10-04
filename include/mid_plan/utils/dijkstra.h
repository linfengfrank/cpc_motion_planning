#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/utils/grid_graph.h>
#include <queue>
#include <unordered_map>

class Dijkstra : public GridGraph
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
  void dijkstra2D_with_line_map(CUDA_GEO::coord glb_tgt, EDTMap* line_map, bool is_path_blocked, float safety_radius);
  void dijkstra2D_with_line(CUDA_GEO::coord glb_tgt, CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b);
  CUDA_GEO::coord find_available_target_with_line(CUDA_GEO::coord start, CUDA_GEO::coord goal, EDTMap* line_map, float safety_radius, const std::unordered_map<int, pathPntInfo> *lpta);
  void dijkstra3D(CUDA_GEO::coord glb_tgt);
  CUDA_GEO::coord get_first_free_coord(CUDA_GEO::coord start, float safety_radius);
  void update_selected_tgt(CUDA_GEO::coord& sel_tgt, float &min_h, int &max_id, const CUDA_GEO::coord &mc, const CUDA_GEO::coord &goal, float safety_radius,
                           EDTMap* line_map, const std::unordered_map<int, pathPntInfo> *lpta);
private:
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;
  bool get_root_idx(int &idx, const CUDA_GEO::coord &mc, EDTMap* line_map);

  bool two_circle_collision(const CUDA_GEO::pos &c_r, const CUDA_GEO::pos &c_f,float safety_radius)
  {
    CUDA_GEO::coord c;
    bool occupied = true;

    c = pos2coord(c_r);
    if (isInside(c))
      getObsCostAndOccupancy(c,0,occupied,false,safety_radius);

    if(occupied)
      return true;

    occupied = true;
    c = pos2coord(c_f);
    if (isInside(c))
      getObsCostAndOccupancy(c,0,occupied,false,safety_radius);

    if(occupied)
      return true;

    return false;
  }

};

#endif // DIJKSTRA_H
