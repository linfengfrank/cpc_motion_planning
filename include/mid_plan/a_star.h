#ifndef A_STAR_H
#define A_STAR_H

#include <mid_plan/grid_graph.h>
class Astar : public GridGraph
{
public:
  Astar(int maxX, int maxY, int maxZ);
  std::vector<CUDA_GEO::coord> AStar2D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone, float &length,
                                       const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_val_map = nullptr);

  std::vector<CUDA_GEO::coord> AStar3D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone, float &length,
                                       const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_val_map = nullptr);

  CUDA_GEO::coord findTargetCoord(const std::vector<CUDA_GEO::coord> &path);
  std::vector<CUDA_GEO::pos> findSplitCoords(const std::vector<CUDA_GEO::coord> &path);
  bool checkTopo(const std::vector<CUDA_GEO::coord> &path_a,const std::vector<CUDA_GEO::coord> &path_b);

};

#endif // A_STAR_H
