#ifndef GRID_GRAPH
#define GRID_GRAPH
#include <cuda_geometry/cuda_edtmap.cuh>
#include <mid_plan/utils/map_base.h>
#include <mid_plan/utils/SortedSet.h>
#include <cpc_aux_mapping/grid_map.h>

#define MID_SAFE_DIST 0.351f
struct nodeInfo
{
  bool inClosed;
  bool inQ;
  float g;
  float h;
  float theta;
  float3 pose;
  float3 action;
  float3 action_lead2me;
  CUDA_GEO::coord c;
  nodeInfo* ptr2parent;
  std::multiset<std::pair<float, nodeInfo*>>::iterator it;
  nodeInfo():
    inClosed(false),
    inQ(false),
    g(std::numeric_limits<float>::infinity()),
    h(std::numeric_limits<float>::infinity()),
    theta(0),
    pose(make_float3(0,0,0)),
    action(make_float3(0,0,0)),
    action_lead2me(make_float3(0,0,0)),
    ptr2parent(nullptr)
  {

  }
};

struct pathPntInfo
{
  float desired_angle;
  int path_idx;
};

class GridGraph : public MapBase
{
public:
  GridGraph(int maxX, int maxY, int maxZ);
  float obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied, bool extend=false, float obstacle_dist = MID_SAFE_DIST) const;
  float obsCostAt(CUDA_GEO::coord s, float default_value, float obstacle_dist = MID_SAFE_DIST) const;
  bool isSeen(const CUDA_GEO::coord & s, const bool default_value) const;
  float getEdt(const CUDA_GEO::coord & s, const float default_value) const;

  nodeInfo* getIdMapPtr() {return _id_map;}
  SeenDist* getEdtMapPtr() {return _val_map;}
  int getMapByteSize() {return _mapByteSize;}
  int getMaxX() {return _w;}
  int getMaxY() {return _h;}
  int getMaxZ() {return _d;}
  float getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const;
  float getTheta(const CUDA_GEO::coord & s, const float &default_value) const;

  void copyIdData(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg);


  int calcTgtHeightCoord(float tgt_height);
  std::vector<CUDA_GEO::coord> rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius = -1);

  float getEdtValue(const CUDA_GEO::coord &c)
  {
    return _val_map[coord2index(c)].d;
  }

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

  inline float dist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2)
  {
    CUDA_GEO::coord c = c1-c2;
    return sqrtf(static_cast<float>(c.square()));
  }

public:
  bool isLOS(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float obstacle_dist = MID_SAFE_DIST)
  {
    bool los = true;
    bool occupied = false;
    for (CUDA_GEO::coord s : rayCast(p0Index, p1Index))
    {
      obsCostAt(s, 0, occupied, obstacle_dist);
      if (occupied)
      {
        los = false;
        break;
      }
    }
    return los;
  }
  //---
  bool isOccupied(const CUDA_GEO::coord &s, float obstacle_dist = MID_SAFE_DIST)
  {
    bool occupied = false;
    obsCostAt(s, 0, occupied, obstacle_dist);
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
  //---
  bool isInside(const CUDA_GEO::pos &p)
  {
    CUDA_GEO::coord s = pos2coord(p);
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=_d)
      return false;

    return true;
  }
  //---
  int lineseg_proj(const CUDA_GEO::pos &seg_a, const CUDA_GEO::pos &seg_b, const CUDA_GEO::pos &vehicle_pos, CUDA_GEO::pos &proj_pnt) const
  {
    CUDA_GEO::pos seg_v=seg_b-seg_a;
    CUDA_GEO::pos pt_v=vehicle_pos-seg_a;
    float seg_v_len = sqrtf(seg_v.square());
    CUDA_GEO::pos seg_v_unit=seg_v/seg_v_len;
    float proj= pt_v.x*seg_v_unit.x + pt_v.y*seg_v_unit.y + pt_v.z*seg_v_unit.z; // dot product
    CUDA_GEO::pos proj_v=seg_v_unit*proj;
    int indicator = 0;
    if (proj <=0)
    {
      proj_pnt = seg_a;
      indicator = -2;
    }
    else if(proj>seg_v_len)
    {
      proj_pnt = seg_b;
      indicator = -1;
    }
    else
    {
      proj_pnt = proj_v+seg_a;
      indicator = 0;
    }
    return indicator;
  }
  //---
  float straight_line_proj(const CUDA_GEO::pos &seg_a, const CUDA_GEO::pos &seg_b, const CUDA_GEO::pos &vehicle_pos) const
  {
    CUDA_GEO::pos seg_v=seg_b-seg_a;
    CUDA_GEO::pos pt_v=vehicle_pos-seg_a;
    float seg_v_len = sqrtf(seg_v.square());
    CUDA_GEO::pos seg_v_unit=seg_v/seg_v_len;
    float proj= pt_v.x*seg_v_unit.x + pt_v.y*seg_v_unit.y + pt_v.z*seg_v_unit.z; // dot product
    CUDA_GEO::pos proj_v=seg_v_unit*proj;
    CUDA_GEO::pos proj_pnt = proj_v+seg_a;

    return sqrtf((vehicle_pos-proj_pnt).square());
  }
};

#endif // GRID_GRAPH
