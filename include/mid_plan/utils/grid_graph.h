#ifndef GRID_GRAPH
#define GRID_GRAPH
#include <cuda_geometry/cuda_edtmap.cuh>
#include <mid_plan/utils/map_base.h>
#include <mid_plan/utils/SortedSet.h>
#include <cpc_aux_mapping/grid_map.h>

#define MID_SAFE_DIST 0.351f
struct nodeInfo
{
  bool inClosed; // Is it in the closed list
  bool inQ; // Is it in the open queue
  float g; // Cost to come
  float h; // Heuristic
  float theta; // Desired yaw angle
  float3 pose; // Vehicle pose
  float3 action; // Moving action
  float3 action_lead2me; // Action that leads to this node
  CUDA_GEO::coord c; // Corresponding coord
  nodeInfo* ptr2parent; // Ptr to parent node
  std::multiset<std::pair<float, nodeInfo*>>::iterator it; // Pointer to heap
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

// A structure to map from a path grid (represented by path_idx to otherdata, like the desired angle at that grid)
struct pathPntInfo
{
  float desired_angle;
  int path_idx;
};

class GridGraph : public MapBase
{
public:
  GridGraph(int maxX, int maxY, int maxZ);
  // TODO: unify the two obstalce cost function
  float getObsCostAndOccupancy(CUDA_GEO::coord s, float default_value, bool &occupied, bool extend=false, float obstacle_dist = MID_SAFE_DIST) const;
  float obsCostAt(CUDA_GEO::coord s, float default_value, float obstacle_dist = MID_SAFE_DIST, float weight = 10.0f) const;

  // Check whether a grid is seen or not
  bool isSeen(const CUDA_GEO::coord & s, const bool default_value) const;

  // Get the EDT value at gird location s
  float getEdt(const CUDA_GEO::coord & s, const float default_value) const;

  // Get the EDT map ptr
  // TODO: remove the function and use a set EDT function instead
  SeenDist* getEdtMapPtr() {return _val_map;}

  // Get the map dimensions
  int getMaxX() {return _w;}
  int getMaxY() {return _h;}
  int getMaxZ() {return _d;}

  // Read the cost to come at grid location s
  float getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const;

  // Read the disred theta value at a grid location s
  float getTheta(const CUDA_GEO::coord & s, const float &default_value) const;

  // Copy Edt data from message
  void copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg);

  // TODO: remove this unused function
  int calcTgtHeightCoord(float tgt_height);

  // Raycase from p0 coord to p1 coord within the limit_radius
  std::vector<CUDA_GEO::coord> rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius = -1);

  // Calculate the centers of the two point vehicle model
  void calculate_bounding_centres(const CUDA_GEO::pos &p, const float &theta, CUDA_GEO::pos &c_r, CUDA_GEO::pos &c_f) const
  {
    CUDA_GEO::pos uni_dir(cosf(theta),sinf(theta),0);
    c_f = p + uni_dir*0.25f;
    c_r = p - uni_dir*0.25f;
  }

public:
  virtual ~GridGraph();

protected:
  SeenDist *_val_map; // Value map, store EDT value
  nodeInfo *_id_map; // Identity map, store Dijkstra related params
  nodeInfo *_init_id_map; // A copy for reset
  SortedSet<nodeInfo*> _PQ; // Priority queue
  int _mapByteSize; // Map size in bytes
  int _w, _h, _d; // The dimension of the grid map

protected:
  // Access a particular node
  nodeInfo *getNode(CUDA_GEO::coord);

  // Map grid coordinate to index
  inline int coord2index(const CUDA_GEO::coord & s) const
  {
    return s.z*_w*_h+s.y*_w+s.x;
  }

  // Calcuate the distance between two grid coordinate
  inline float dist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2)
  {
    CUDA_GEO::coord c = c1-c2;
    return sqrtf(static_cast<float>(c.square()));
  }

public:
  // Check two coord are LOS or not
  bool isLOS(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float obstacle_dist = MID_SAFE_DIST)
  {
    bool los = true;
    bool occupied = false;
    for (CUDA_GEO::coord s : rayCast(p0Index, p1Index))
    {
      getObsCostAndOccupancy(s, 0, occupied, obstacle_dist);
      if (occupied)
      {
        los = false;
        break;
      }
    }
    return los;
  }

  // Check whether a coord is obstacle free or not
  bool isOccupied(const CUDA_GEO::coord &s, float obstacle_dist = MID_SAFE_DIST)
  {
    bool occupied = false;
    getObsCostAndOccupancy(s, 0, occupied, obstacle_dist);
    return occupied;
  }

  // Check whether a coord is inside the local map area or not
  bool isInside(const CUDA_GEO::coord &s)
  {
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=_d)
      return false;

    return true;
  }

  // Check wehter a pos is inside the local map or not
  bool isInside(const CUDA_GEO::pos &p)
  {
    return isInside(pos2coord(p));
  }

  //Project the vehicle's position to a straight line passing through seg_a and seg_b
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
