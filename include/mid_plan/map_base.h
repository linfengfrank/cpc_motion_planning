#ifndef MAP_BASE_H
#define MAP_BASE_H
#include <cuda_geometry/cuda_edtmap.cuh>
class MapBase
{
public:
  MapBase()
  {
    // Initialize the origin and grid step
    _origin.x = 0;
    _origin.y = 0;
    _origin.z = 0;
    _gridstep = 0.2f;
  }
  //---
  ~MapBase()
  {

  }
  //---
  void setMapSpecs(const CUDA_GEO::pos &origin, const float &gridstep)
  {
    _origin = origin;
    _gridstep = gridstep;
  }
  //---
  void setOrigin(const CUDA_GEO::pos &origin)
  {
    _origin = origin;
  }
  //---
  CUDA_GEO::pos getOrigin() const
  {
    return _origin;
  }
  //---
  float getGridStep() const
  {
    return _gridstep;
  }
  //---
  CUDA_GEO::coord pos2coord(const CUDA_GEO::pos & p) const
  {
    CUDA_GEO::coord output;
    output.x = floorf( (p.x - _origin.x) / _gridstep + 0.5f);
    output.y = floorf( (p.y - _origin.y) / _gridstep + 0.5f);
    output.z = floorf( (p.z - _origin.z) / _gridstep + 0.5f);
    return output;
  }
  //---
  CUDA_GEO::pos coord2pos(const CUDA_GEO::coord & c) const
  {
    CUDA_GEO::pos output;
    output.x = c.x * _gridstep + _origin.x;
    output.y = c.y * _gridstep + _origin.y;
    output.z = c.z * _gridstep + _origin.z;

    return output;
  }
protected:
  CUDA_GEO::pos _origin;
  float _gridstep;
};
#endif // MAP_BASE_H

