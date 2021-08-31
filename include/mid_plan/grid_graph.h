#ifndef GRID_GRAPH
#define GRID_GRAPH
#include <cuda_geometry/cuda_edtmap.cuh>
#include <distmap/map_base.h>
#include <mid_plan/SortedSet.h>
#include <cpc_aux_mapping/grid_map.h>

struct nodeInfo
{
  bool inClosed;
  bool inQ;
  float g;
  float h;
  CUDA_GEO::coord c;
  nodeInfo* ptr2parent;
  std::multiset<std::pair<float, nodeInfo*>>::iterator it;
  nodeInfo():
    inClosed(false),
    inQ(false),
    g(std::numeric_limits<float>::infinity()),
    h(std::numeric_limits<float>::infinity()),
    ptr2parent(nullptr)
  {

  }
};

class GridGraph : public MapBase
{
public:
  GridGraph(int maxX, int maxY, int maxZ);
  float getObsCostAndCollision(CUDA_GEO::coord s, float default_value, bool &occupied, float obstacle_dist) const;
  bool isSeen(const CUDA_GEO::coord & s, const bool default_value) const;

  nodeInfo* getIdMapPtr() {return _id_map;}
  SeenDist* getEdtMapPtr() {return _val_map;}
  float getEdt(int x, int y, int z, float default_value = 0.0f) const
  {
    if (x<0 || x>=_w || y<0 || y>=_h || z<0 || z>=_d)
    {
      return default_value;
    }
    else
    {
      return _val_map[coord2index(x,y,z)].d * _gridstep;
    }
  }

  int getMapByteSize() {return _mapByteSize;}
  int getMaxX() {return _w;}
  int getMaxY() {return _h;}
  int getMaxZ() {return _d;}
  float getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const;

  void copyIdData(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg);


  int calcTgtHeightCoord(float tgt_height);
  std::vector<CUDA_GEO::coord> rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius = -1);

public:
  virtual ~GridGraph();

protected:
  SeenDist *_val_map; // Value map, store EDT value
  nodeInfo *_id_map; // Identity map, store Dijkstra related params
  nodeInfo *_init_id_map; // A copy for reset
  SortedSet<nodeInfo*> _PQ;
  int _mapByteSize;
  int _w, _h, _d;

protected:
  nodeInfo *getNode(CUDA_GEO::coord);
  inline int coord2index(const CUDA_GEO::coord & s) const
  {
    return s.z*_w*_h+s.y*_w+s.x;
  }

  inline int coord2index(int x, int y, int z) const
  {
    return z*_w*_h+y*_w+x;
  }

  inline float dist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2)
  {
    CUDA_GEO::coord c = c1-c2;
    return sqrtf(static_cast<float>(c.square()));
  }

public:
  bool isLOS(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float obstacle_dist = 1.0)
  {
    bool los = true;
    bool occupied = false;
    for (CUDA_GEO::coord s : rayCast(p0Index, p1Index))
    {
      getObsCostAndCollision(s, 0, occupied, obstacle_dist);
      if (occupied)
      {
        los = false;
        break;
      }
    }
    return los;
  }
  //---
  bool isOccupied(const CUDA_GEO::coord &s, float obstacle_dist = 1.0)
  {
    bool occupied = false;
    getObsCostAndCollision(s, 0, occupied, obstacle_dist);
    return occupied;
  }
  //---
  bool isInside(const CUDA_GEO::coord &s)
  {
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=_d)
      return false;

    return true;
  }
};

#endif // GRID_GRAPH
