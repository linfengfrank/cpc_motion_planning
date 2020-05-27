#ifndef DIJKSTRA_H
#define DIJKSTRA_H
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

class DijkstraMap : public MapBase
{
public:
    DijkstraMap(int maxX, int maxY, int maxZ);
    float obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied,
                     const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_val_map = nullptr, float obstacle_dist = 1.0) const;
    bool isSeen(const CUDA_GEO::coord & s, const bool default_value) const;

    nodeInfo* getIdMapPtr() {return _id_map;}
    SeenDist* getEdtMapPtr() {return _val_map;}
    int getMapByteSize() {return _mapByteSize;}
    int getMaxX() {return _w;}
    int getMaxY() {return _h;}
    int getMaxZ() {return _d;}
    float getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const;

    void copyIdData(const cpc_aux_mapping::grid_map::ConstPtr &msg);
    void copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg);

    std::vector<CUDA_GEO::coord> AStar2D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone, float &length,
                                       const CUDA_GEO::coord *crd_shift = nullptr, SeenDist *last_val_map = nullptr);
    CUDA_GEO::coord findTargetCoord(const std::vector<CUDA_GEO::coord> &path);
    int calcTgtHeightCoord(float tgt_height);
    std::vector<CUDA_GEO::coord> rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius = -1);
    std::vector<CUDA_GEO::pos> findSplitCoords(const std::vector<CUDA_GEO::coord> &path);
    bool checkTopo(const std::vector<CUDA_GEO::coord> &path_a,const std::vector<CUDA_GEO::coord> &path_b);
    void dijkstra2D(CUDA_GEO::coord glb_tgt);
public:
    virtual ~DijkstraMap();

private:
    SeenDist *_val_map; // Value map, store EDT value
    nodeInfo *_id_map; // Identity map, store Dijkstra related params
    nodeInfo *_init_id_map; // A copy for reset
    SortedSet<nodeInfo*> _PQ;
    int _mapByteSize;
    int _w, _h, _d;

private:
    nodeInfo *getNode(CUDA_GEO::coord);
    inline int coord2index(const CUDA_GEO::coord & s) const
    {
        return s.z*_w*_h+s.y*_w+s.x;
    }

    inline float dist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2)
    {
        CUDA_GEO::coord c = c1-c2;
        return sqrt(static_cast<float>(c.square()));
    }

    inline float point2lineDist(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2, const CUDA_GEO::coord & c0)
    {
        CUDA_GEO::coord a = c1-c0;
        CUDA_GEO::coord b = c2-c1;
        int a_dot_b = a.x*b.x + a.y*b.y + a.z*b.z;

        if (b.square() == 0)
            return sqrt(static_cast<float>(a.square()));

        return sqrt(static_cast<float>(a.square()*b.square() - a_dot_b*a_dot_b)/static_cast<float>(b.square()));
    }

public:
    bool isLOS(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index)
    {
        bool los = true;
        bool occupied = false;
        for (CUDA_GEO::coord s : rayCast(p0Index, p1Index))
        {
            obsCostAt(s, 0, occupied);
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
       obsCostAt(s, 0, occupied, nullptr,nullptr, obstacle_dist);
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

#endif // DIJKSTRA_H
