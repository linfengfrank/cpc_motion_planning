#include "mid_plan/DijkstraMap.h"
#include <string.h>
#include <chrono>
#include <ros/console.h>
#include <stack>
DijkstraMap::DijkstraMap(int maxX, int maxY, int maxZ):
    MapBase(),
    _w(maxX),
    _h(maxY),
    _d(maxZ)
{
    // Value map to store the seenDist map received from the edt node
    _val_map = new SeenDist[_w*_h*_d];
    memset(_val_map, 0, _w*_h*_d*sizeof(SeenDist));

    // Identity map used by the Dijkstra's algorithm
    _id_map = new nodeInfo[_w*_h*_d];
    _init_id_map = new nodeInfo[_w*_h*_d];
    _mapByteSize = _w*_h*_d*sizeof(nodeInfo);

    //Assign coords value for all members in the id map
    CUDA_GEO::coord s;
    for (s.x=0; s.x<_w; s.x++)
    {
        for (s.y=0; s.y<_h; s.y++)
        {
            for (s.z=0; s.z<_d; s.z++)
            {
                _init_id_map[coord2index(s)].c=s;
            }
        }
    }
}

DijkstraMap::~DijkstraMap()
{
    delete [] _val_map;
    delete [] _id_map;
    delete [] _init_id_map;
}

float DijkstraMap::getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const
{
    float g=0.0;
    if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=_d)
    {
        return default_value;
    }
    else
    {
        return _id_map[coord2index(s)].g;
    }
}

float DijkstraMap::obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied,
                              const CUDA_GEO::coord *crd_shift, SeenDist *last_val_map, float obstacle_dist) const
{
    SeenDist* map_ptr;
    if (last_val_map != nullptr)
    {
        s = s + (*crd_shift);
        default_value = 100;
        map_ptr = last_val_map;
    }
    else
    {
        map_ptr = _val_map;
    }

    float dist = 0.0f;
    float cost = 0.0;
    if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=_d)
    {
        dist = default_value;
    }
    else
    {
        dist = map_ptr[coord2index(s)].d;
    }
    dist *= static_cast<float>(getGridStep());

    // Check wheter it is on any obstacle or stays in the height range
    if (dist <= obstacle_dist)
    {
        cost = 400.0*exp(-dist*1.5);
        occupied = true;
    }
    else
    {
        cost = 0.0;
        occupied = false;
    }

    return cost;
}

nodeInfo* DijkstraMap::getNode(CUDA_GEO::coord s)
{
    if (s.x<0 || s.x>=_w ||
            s.y<0 || s.y>=_h ||
            s.z<0 || s.z>=_d)
        return nullptr;

    return &_id_map[coord2index(s)];
}

bool DijkstraMap::isSeen(const CUDA_GEO::coord & s, const bool default_value) const
{
    if (s.x<0 || s.x>=_w ||
            s.y<0 || s.y>=_h ||
            s.z<0 || s.z>=_d)
        return default_value;

    return _val_map[coord2index(s)].s;
}

void DijkstraMap::copyIdData(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
    if (msg->x_size != _w || msg->y_size != _h || msg->z_size != _d)
    {
        printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
        return;
    }

    setMapSpecs(CUDA_GEO::pos(static_cast<float>(msg->x_origin),
                            static_cast<float>(msg->y_origin),
                            static_cast<float>(msg->z_origin)), msg->width);
    memcpy (_id_map, msg->payload8.data(), sizeof(nodeInfo)*_w*_h*_d);
}

void DijkstraMap::copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
    if (msg->x_size != _w || msg->y_size != _h || msg->z_size != _d)
    {
        printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
        return;
    }

    setMapSpecs(CUDA_GEO::pos(static_cast<float>(msg->x_origin),
                            static_cast<float>(msg->y_origin),
                            static_cast<float>(msg->z_origin)), msg->width);
    memcpy (_val_map, msg->payload8.data(), sizeof(SeenDist)*_w*_h*_d);
}

std::vector<CUDA_GEO::coord> DijkstraMap::rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius)
{
    std::vector<CUDA_GEO::coord> output;
    CUDA_GEO::pos p0 = coord2pos(p0Index);
    // p0 must be inside the map area
    if(p0Index.x < 0 || p0Index.x > _w-1 ||
       p0Index.y < 0 || p0Index.y > _h-1 ||
       p0Index.z < 0 || p0Index.z > _d-1)
    {
        ROS_ERROR("p0 is not inside the map.");
        exit(-1);
    }

    CUDA_GEO::pos p1 = coord2pos(p1Index);

    // same grid, we are done
    if(p0Index.x == p1Index.x && p0Index.y == p1Index.y && p0Index.z == p1Index.z)
    {
        output.push_back(p0Index);
        return output;
    }
    // Initialization phase ------------------------
    CUDA_GEO::pos direction = p1 - p0;
    float length = sqrt(direction.square());
    direction = direction / length; //normalize the vector

    int    step[3];
    float tMax[3];
    float tDelta[3];

    CUDA_GEO::coord currIndex = p0Index;
    for (unsigned int i=0; i<3; i++)
    {
        // compute step direction
        if(direction.at(i) > 0.0) step[i] = 1;
        else if (direction.at(i) < 0.0) step[i] = -1;
        else step[i] = 0;

        // compute tMax, tDelta
        //tMax: shortest route? tDelta?  voxelBorder=pos.x?  pos: /m   coord: idx of grid
        if(step[i] != 0)
        {
            float voxelBorder = float(currIndex.at(i)) * _gridstep +
                    _origin.const_at(i) + float(step[i]) * _gridstep*0.5;
            tMax[i] = (voxelBorder - p0.const_at(i))/direction.at(i);
            tDelta[i] = _gridstep / fabs(direction.at(i));
        }
        else
        {
            tMax[i] =  std::numeric_limits<float>::max( );
            tDelta[i] = std::numeric_limits<float>::max( );
        }
    }

    // Incremental phase -----------------------------------
    bool done = false;
    while (!done)
    {
        output.push_back(currIndex);
        // reached endpoint? or go outside map boundary?
        if( (currIndex.x == p1Index.x && currIndex.y == p1Index.y && currIndex.z == p1Index.z) ||
                currIndex.x <= 0 || currIndex.x >= _w-1 ||
                currIndex.y <= 0 || currIndex.y >= _h-1 ||
                currIndex.z <= 0 || currIndex.z >= _d-1)
        {
            done = true;
            break;
        }

        if (limit_radius > 0 && sqrtf((currIndex-p0Index).square())*getGridStep() > limit_radius)
        {
          done = true;
          break;
        }

        unsigned int dim;

        // find minimum tMax;
        if (tMax[0] < tMax[1]){
            if (tMax[0] < tMax[2]) dim = 0;
            else                   dim = 2;
        }
        else {
            if (tMax[1] < tMax[2]) dim = 1;
            else                   dim = 2;
        }

        // advance in drection "dim"
        currIndex.at(dim) += step[dim];
        tMax[dim] += tDelta[dim];
    }
    return output;
}

std::vector<CUDA_GEO::coord> DijkstraMap::AStar2D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone,  float &length,
                                                const CUDA_GEO::coord *crd_shift, SeenDist *last_val_map)
{
    std::vector<CUDA_GEO::coord> output;

    //First clear the old information
    memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*_w*_h*_d);
    _PQ.clear();

    // Useful variables
    bool occupied = false;
    CUDA_GEO::coord my_coord, child_coord, closest_coord;
    float min_h = std::numeric_limits<float>::infinity();

    //Set the root
    nodeInfo* ni;
    ni=getNode(start);
    if (ni)
    {
        ni->g = 0;
        ni->h = dist(ni->c,goal)*getGridStep();
        _PQ.insert(ni,ni->g + ni->h);
        closest_coord = ni->c;
        min_h = ni->h;
    }
    else
    {
        ROS_ERROR("First point outside the boundary.");
        exit(-1);
    }

    while (_PQ.size()>0)
    {
        ni=_PQ.pop();
        ni->inClosed = true;
        my_coord = ni->c;

        obsCostAt(my_coord,0,occupied,crd_shift,last_val_map);
        if (!occupied)
            reached_free_zone = true;

        if (min_h > ni->h)
        {
            min_h = ni->h;
            closest_coord = ni->c;
        }

        if (my_coord == goal)
            break;

        // get all neighbours
        for (int ix=-1;ix<=1;ix++)
        {
            for (int iy=-1;iy<=1;iy++)
            {
                if (ix==0 && iy ==0)
                    continue;

                child_coord.x = my_coord.x + ix;
                child_coord.y = my_coord.y + iy;
                child_coord.z = my_coord.z;
                nodeInfo* p = getNode(child_coord);
                if (p && !p->inClosed)
                {

                    float new_g = sqrt(static_cast<float>(ix*ix+iy*iy))*getGridStep() +
                            ni->g + obsCostAt(child_coord,0,occupied,crd_shift,last_val_map);
                    if (p->g > new_g && !(occupied && reached_free_zone))
                    {
                        p->ptr2parent = ni;
                        p->g = new_g;
                        p->h = dist(p->c, goal)*getGridStep();
                        _PQ.insert(p,p->g + p->h);
                    }
                }
            }
        }
    }

    //retrieve the path
    ni = getNode(closest_coord);
    length = 0;
    CUDA_GEO::coord shift;
    while(ni != nullptr)
    {
        child_coord = ni->c;
        output.push_back(ni->c);
        ni = ni->ptr2parent;
        if (ni != nullptr)
        {
            my_coord = ni->c;
            shift = my_coord-child_coord;
            length += sqrt(static_cast<float>(shift.square()))*_gridstep;
        }
    }

    if (output.size() == 0)
    {
        ROS_ERROR("Path size is zero.");
        exit(-1);
    }

    return output;
}

int DijkstraMap::calcTgtHeightCoord(float tgt_height)
{
    int coord = floor( (tgt_height - _origin.z) / _gridstep + 0.5);

    if (coord < 0)
        coord = 0;

    if (coord > _d - 1 )
        coord = _d -1;

    return coord;
}

CUDA_GEO::coord DijkstraMap::findTargetCoord(const std::vector<CUDA_GEO::coord> &path)
{
    // If there is no path, return the center point
    if (path.size() == 0)
        return CUDA_GEO::coord(_w/2,_h/2,_d/2);

    unsigned int anchor = path.size() - 1;
    unsigned int target = 0;

    for (int i=0; i<path.size(); i++)
    {
        // find the largest distance
        int max_id = 0;
        float max_dist = 0;
        for (int j = target; j< anchor; j++)
        {
            float dist = point2lineDist(path[anchor],path[target],path[j]);
            if (dist > max_dist)
            {
                max_dist = dist;
                max_id = j;
            }
        }

        if (max_dist*_gridstep > 2.5)
        {
            target = max_id;
        }
        else
        {
            break;
        }
    }

    return path[target];
}

std::vector<CUDA_GEO::pos> DijkstraMap::findSplitCoords(const std::vector<CUDA_GEO::coord> &path)
{
    std::vector<CUDA_GEO::pos> split_pts;
    std::stack<std::pair<int,int>> pls; // fist is anchor, second is target
    pls.push(std::make_pair(path.size() - 1, 0));

    int check = path.size() - 1;

    while (pls.size() > 0)
    {
        int target = pls.top().second;
        int anchor = pls.top().first;
        pls.pop();
        // find the largest distance
        int max_id = 0;
        float max_dist = 0;
        for (int j = target; j< anchor; j++)
        {
            float dist = point2lineDist(path[anchor],path[target],path[j]);
            if (dist > max_dist)
            {
                max_dist = dist;
                max_id = j;
            }
        }

        if (max_dist*_gridstep > 0.5)
        {
            pls.push(std::make_pair(max_id, target));
            pls.push(std::make_pair(anchor, max_id));
            target = max_id;
        }
        else
        {
            split_pts.push_back(coord2pos(path[target]));
            if (target >= check)
            {
                ROS_ERROR("Wrong split sequence");
                exit(-1);
            }
            check = target;
        }
    }
    std::cout<<std::endl;
    return split_pts;
}

bool DijkstraMap::checkTopo(const std::vector<CUDA_GEO::coord> &path_a,const std::vector<CUDA_GEO::coord> &path_b)
{
  int K=20;
  for (int i=0; i<K; i++)
  {
    int idx_a = static_cast<float>(i)/K*path_a.size();
    int idx_b = static_cast<float>(i)/K*path_b.size();
    std::vector<CUDA_GEO::coord> line = rayCast(path_a[idx_a], path_b[idx_b]);
    for (CUDA_GEO::coord &c : line)
    {
      if(isOccupied(c,1.5f))
        return false;
    }
  }
  return true;
}

void DijkstraMap::dijkstra2D(CUDA_GEO::coord glb_tgt)
{
    memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*_w*_h*_d);
    //Get the local Dijkstra target
    CUDA_GEO::coord mc = glb_tgt;
    CUDA_GEO::coord pc;
    nodeInfo* m;
    // insert the root
    m=getNode(mc);

    bool occupied=false;
    if (m)
    {
        m->g = 0 + obsCostAt(mc,0,occupied);
        _PQ.insert(m,m->g);
    }
    auto start = std::chrono::steady_clock::now();
    while (_PQ.size()>0)
    {
        m=_PQ.pop();
        m->inClosed = true;
        mc = m->c;

        // get all neighbours
        for (int ix=-1;ix<=1;ix++)
        {
          for (int iy=-1;iy<=1;iy++)
          {
            if (ix==0 && iy ==0)
              continue;

            pc.x = mc.x + ix;
            pc.y = mc.y + iy;
            pc.z = mc.z;
            nodeInfo* p = getNode(pc);
            if (p)
            {
              if (!p->inClosed)
              {
                double new_g = sqrt(static_cast<double>(ix*ix+iy*iy))*getGridStep() +
                    m->g + obsCostAt(pc,0,occupied);
                if (p->g > new_g)
                {
                  p->g = new_g;
                  _PQ.insert(p,p->g);
                }
              }
            }
          }
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << "Planning time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
}

