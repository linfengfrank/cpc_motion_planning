#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H
#include <mid_plan/utils/grid_graph.h>

class PathSmoother
{
public:
  PathSmoother(GridGraph *gh);
  void set_path(const std::vector<CUDA_GEO::pos> &path)
  {
    m_path.clear();
    for (size_t i=0; i<path.size(); i++)
    {
      m_path.push_back(make_float2(path[i].x, path[i].y));
    }
  }

  std::vector<CUDA_GEO::pos> get_path()
  {
    std::vector<CUDA_GEO::pos> output;
    for (size_t i=0; i<m_path.size(); i++)
    {
      output.push_back(CUDA_GEO::pos(m_path[i].x, m_path[i].y, 0));
    }
    return output;
  }

private:
  GridGraph* m_gh;
  std::vector<float2> m_path;

private:

  void smooth_path();
  void apply_eb();
  void autoResize();

  // Helper functions
  float2 get_gradient(const float2 &p)
  {
    CUDA_GEO::coord c = m_gh->pos2coord(CUDA_GEO::pos(p.x,p.y,0));
    float2 grad = make_float2(0,0);
    CUDA_GEO::coord shift;

    for (shift.x=-1; shift.x<=1; shift.x++)
    {
      for (shift.y=-1; shift.y<=1; shift.y++)
      {
        if (shift.x == 0 && shift.y ==0)
          continue;

        grad += get_gradient(c, c+shift);
      }
    }
    return grad;
  }

  float2 get_gradient(const CUDA_GEO::coord &ctr, const CUDA_GEO::coord &oth)
  {
    float cost_ctr = 80*expf(-0.5f*m_gh->getGridStep()*m_gh->getEdtValue(ctr));
    float cost_oth = 80*expf(-0.5f*m_gh->getGridStep()*m_gh->getEdtValue(oth));
    CUDA_GEO::coord diff = oth - ctr;
    float2 delta = make_float2(diff.x,diff.y);
    return (cost_ctr - cost_oth)*delta;
  }


  float dist(const float2 &a, const float2&b)
  {
    return sqrtf(dot(a-b,a-b));
  }

  float2 get_force(const float2 &ctr, const float2 &magnet)
  {
    return magnet - ctr;
  }
};

#endif // PATH_SMOOTHER_H
