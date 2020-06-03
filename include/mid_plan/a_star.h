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

};

#endif // A_STAR_H
