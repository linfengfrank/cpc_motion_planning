#ifndef MAPCOSTFUNCTION_H
#define MAPCOSTFUNCTION_H

#include <vector>
#include "ceres/ceres.h"
#include "mid_plan/grid_graph.h"

namespace SPL_OPTI
{
class MapCostFunction : public ceres::SizedCostFunction<1,1,1,1>
{
public:
  MapCostFunction(double w, double r, GridGraph* map)
  {
    m_w = w;
    m_r = r;
    m_map = map;
    resolution_inv_ = 1/m_map->getGridStep();
  }

  inline void assign_values(double* F,
                            double** J,
                            const double &res,
                            const double &j0,
                            const double &j1,
                            const double &j2) const
  {
    // Residule
    F[0] = res;

    // Jacobian
    if (J != nullptr)
    {
      if (J[0] != nullptr)
        J[0][0] = j0;

      if (J[1] != nullptr)
        J[1][0] = j1;

      if (J[2] != nullptr)
        J[2][0] = j2;
    }
  }

  virtual bool Evaluate(double const* const* parameters,
                        double* F,
                        double** J) const
  {
    // 3D case
    // Get the point we are examining
    CUDA_GEO::pos p(parameters[0][0],parameters[1][0],parameters[2][0]);

    // Regulate this point
    CUDA_GEO::pos pr = m_map->regulate_pos(p);

    // Get the left bottom corner index
    CUDA_GEO::coord lb(floorf(pr.x), floorf(pr.y), floorf(pr.z));

    // Get the difference vector
    CUDA_GEO::pos diff(pr.x-lb.x, pr.y-lb.y, pr.z-lb.z);

    double values[2][2][2];
    // Get surronding neighbour EDT values
    get_neighbour_edt(values,lb);

    // trilinear interpolation
    double v00 = (1 - diff.x) * values[0][0][0] + diff.x * values[1][0][0];
    double v01 = (1 - diff.x) * values[0][0][1] + diff.x * values[1][0][1];
    double v10 = (1 - diff.x) * values[0][1][0] + diff.x * values[1][1][0];
    double v11 = (1 - diff.x) * values[0][1][1] + diff.x * values[1][1][1];

    double v0 = (1 - diff.y) * v00 + diff.y * v10;
    double v1 = (1 - diff.y) * v01 + diff.y * v11;

    double dist = (1 - diff.z) * v0 + diff.z * v1;

    if (dist < m_r +0.05)
    {
      // Get the gradient
      double grad[3];
      grad[2] = (v1 - v0) * resolution_inv_;
      grad[1] = ((1 - diff.z) * (v10 - v00) + diff.z * (v11 - v01)) * resolution_inv_;
      grad[0] = (1 - diff.z) * (1 -diff.y) * (values[1][0][0] - values[0][0][0]);
      grad[0] += (1 - diff.z) *diff.y * (values[1][1][0] - values[0][1][0]);
      grad[0] += diff.z * (1 -diff.y) * (values[1][0][1] - values[0][0][1]);
      grad[0] += diff.z *diff.y * (values[1][1][1] - values[0][1][1]);
      grad[0] *= resolution_inv_;

      assign_values(F,J,(m_r+0.05-dist)*m_w,-m_w*grad[0],-m_w*grad[1],-m_w*grad[2]);
    }
    else
    {
      assign_values(F,J,0,0,0,0);
    }


    return true;
  }

  void get_neighbour_edt(double v[2][2][2], const CUDA_GEO::coord &lb) const
  {
    for (int x=0; x<=1; x++)
    {
      for (int y=0; y<=1; y++)
      {
        for (int z=0; z<=1; z++)
        {
          v[x][y][z] = m_map->getEdt(lb.x+x,lb.y+y,lb.z+z);
        }
      }
    }
  }

private:
  double m_w;
  double m_r;
  double resolution_inv_;
  GridGraph *m_map;
};
}
#endif // MAPCOSTFUNCTION_H
