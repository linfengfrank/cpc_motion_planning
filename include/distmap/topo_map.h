#ifndef LINDISTMAP_H
#define LINDISTMAP_H
#include <distmap/map_base.h>
#include <iostream>
#include <cpc_aux_mapping/grid_map.h>

class TopoMap : public MapBase
{
public:
  TopoMap(int maxX, int maxY, int maxZ);
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
  virtual ~TopoMap();

private:
  SeenDist *_map;
  int _w, _h, _d;
};

#endif // LINDISTMAP_H
