#ifndef LINDISTMAP_H
#define LINDISTMAP_H
#include <cuda_geometry/cuda_edtmap.cuh>
#include <iostream>
#include <cpc_aux_mapping/grid_map.h>
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

class LinDistMap : public MapBase
{
public:
  LinDistMap(int maxX, int maxY, int maxZ);
  CUDA_GEO::pos findNearestPoint(CUDA_GEO::pos _p, float tor_min_dist, bool valid_z) const;
  inline CUDA_GEO::coord transformPosToCoord(CUDA_GEO::pos& _p) const{
    CUDA_GEO::pos _origin = this->getOrigin();
    float _res = this->getGridStep();
    CUDA_GEO::coord _s;
    _s.x = std::floor((_p.x - _origin.x)/_res);
    _s.y = std::floor((_p.y - _origin.y)/_res);
    _s.z = std::floor((_p.z - _origin.z)/_res);
    return _s;
  }
  inline CUDA_GEO::pos transformCoordToPos(CUDA_GEO::coord& _c) const{
    CUDA_GEO::pos _origin = this->getOrigin();
    float _res = this->getGridStep();
    CUDA_GEO::pos _p;
    _p.x = _origin.x + static_cast<float>(_c.x)*_res + _res/2.0;
    _p.y = _origin.y + static_cast<float>(_c.y)*_res + _res/2.0;
    _p.z = _origin.z + static_cast<float>(_c.z)*_res + _res/2.0;
    return _p;
  }
  float distAt(const CUDA_GEO::coord & s, const float default_value) const;
  float distAt(const float&x, const float&y, const float&z, const float default_value) const;
  void distWithGrad(const CUDA_GEO::pos& pt, float default_value, float& dist, CUDA_GEO::pos& grad) const;
  CUDA_GEO::pos indexToPos(const CUDA_GEO::coord & s) const;
  bool isSeen(const CUDA_GEO::coord & s, const bool default_value) const;
  bool isSeen(const float&x, const float&y, const float&z, const bool default_value) const;

  SeenDist* getMapPtr() {return _map;}
  int getMaxX() {return _w;}
  int getMaxY() {return _h;}
  int getMaxZ() {return _d;}

  void copyData(const cpc_aux_mapping::grid_map &msg);

public:
  virtual ~LinDistMap();

private:
  SeenDist *_map;
  int _w, _h, _d;
};

#endif // LINDISTMAP_H
