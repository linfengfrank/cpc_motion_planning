#include "mid_plan/utils/grid_graph.h"
#include <string.h>
#include <chrono>
#include <ros/console.h>
GridGraph::GridGraph(int maxX, int maxY, int maxZ):
  MapBase(),
  _w(maxX),
  _h(maxY),
  _d(maxZ)
{
  // Value map to store the seenDist map received from the edt node
  _val_map = new SeenDist[_w*_h*_d];
  memset(_val_map, 0, static_cast<size_t>(_w*_h*_d)*sizeof(SeenDist));

  // Identity map used by the Dijkstra's algorithm
  _id_map = new nodeInfo[_w*_h*_d];
  _init_id_map = new nodeInfo[_w*_h*_d];
  _mapByteSize = static_cast<int>(static_cast<size_t>(_w*_h*_d)*sizeof(nodeInfo));

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

GridGraph::~GridGraph()
{
  delete [] _val_map;
  delete [] _id_map;
  delete [] _init_id_map;
}

float GridGraph::getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const
{
  if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=_d)
  {
    return default_value;
  }
  else
  {
    return _id_map[coord2index(s)].g;
  }
}

float GridGraph::getTheta(const CUDA_GEO::coord & s, const float &default_value) const
{
  if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=_d)
  {
    return default_value;
  }
  else
  {
    return _id_map[coord2index(s)].theta;
  }
}

float GridGraph::obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied, bool extend, float obstacle_dist) const
{
  SeenDist* map_ptr;
  map_ptr = _val_map;

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
    cost = 400.0f*expf(-dist*1.5f);
    occupied = true;
  }
  else
  {
    if (extend && dist <= obstacle_dist + 0.5f)
      cost = 6.0f*expf(-dist*3.6f);
    else
      cost = 0;
    occupied = false;
  }

  return cost;
}

float GridGraph::obsCostAt(CUDA_GEO::coord s, float default_value, float obstacle_dist) const
{
  SeenDist* map_ptr;
  map_ptr = _val_map;

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

  float free_dist = fmaxf(dist - obstacle_dist, 0);
  cost += expf(-9.0f*free_dist)*50;

  //if (dist < obstacle_dist + 1.0f)
  //  cost += expf(-4.0f*dist)*4;

  if (dist < obstacle_dist)
    cost += 1000;
  return cost;
}

nodeInfo* GridGraph::getNode(CUDA_GEO::coord s)
{
  if (s.x<0 || s.x>=_w ||
      s.y<0 || s.y>=_h ||
      s.z<0 || s.z>=_d)
    return nullptr;

  return &_id_map[coord2index(s)];
}

bool GridGraph::isSeen(const CUDA_GEO::coord & s, const bool default_value) const
{
  if (s.x<0 || s.x>=_w ||
      s.y<0 || s.y>=_h ||
      s.z<0 || s.z>=_d)
    return default_value;

  return _val_map[coord2index(s)].s;
}

float GridGraph::getEdt(const CUDA_GEO::coord & s, const float default_value) const
{
  if (s.x<0 || s.x>=_w ||
      s.y<0 || s.y>=_h ||
      s.z<0 || s.z>=_d)
    return default_value;

  return _val_map[coord2index(s)].d * _gridstep;
}

void GridGraph::copyIdData(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  if (msg->x_size != _w || msg->y_size != _h || msg->z_size != _d)
  {
    printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
    return;
  }

  setMapSpecs(CUDA_GEO::pos(static_cast<float>(msg->x_origin),
                            static_cast<float>(msg->y_origin),
                            static_cast<float>(msg->z_origin)), msg->width);
  memcpy (_id_map, msg->payload8.data(), sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
}

void GridGraph::copyEdtData(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  if (msg->x_size != _w || msg->y_size != _h || msg->z_size != _d)
  {
    printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
    return;
  }

  setMapSpecs(CUDA_GEO::pos(static_cast<float>(msg->x_origin),
                            static_cast<float>(msg->y_origin),
                            static_cast<float>(msg->z_origin)), msg->width);
  memcpy (_val_map, msg->payload8.data(), sizeof(SeenDist)*static_cast<size_t>(_w*_h*_d));
}

std::vector<CUDA_GEO::coord> GridGraph::rayCast(const CUDA_GEO::coord &p0Index, const CUDA_GEO::coord &p1Index, float limit_radius)
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
  float length = sqrtf(direction.square());
  direction = direction / length; //normalize the vector

  int    step[3];
  float tMax[3];
  float tDelta[3];

  CUDA_GEO::coord currIndex = p0Index;
  for (int i=0; i<3; i++)
  {
    // compute step direction
    if(direction.at(i) > 0.0f) step[i] = 1;
    else if (direction.at(i) < 0.0f) step[i] = -1;
    else step[i] = 0;

    // compute tMax, tDelta
    //tMax: shortest route? tDelta?  voxelBorder=pos.x?  pos: /m   coord: idx of grid
    if(step[i] != 0)
    {
      float voxelBorder = float(currIndex.at(i)) * _gridstep +
          _origin.const_at(i) + float(step[i]) * _gridstep*0.5f;
      tMax[i] = (voxelBorder - p0.const_at(i))/direction.at(i);
      tDelta[i] = _gridstep / fabsf(direction.at(i));
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

    // reached endpoint? or go outside map boundary?
    if( (currIndex.x == p1Index.x && currIndex.y == p1Index.y && currIndex.z == p1Index.z) ||
        currIndex.x < 0 || currIndex.x > _w-1 ||
        currIndex.y < 0 || currIndex.y > _h-1 ||
        currIndex.z < 0 || currIndex.z > _d-1)
    {
      done = true;
      break;
    }

    output.push_back(currIndex);

    if (limit_radius > 0 && sqrtf((currIndex-p0Index).square())*getGridStep() > limit_radius)
    {
      done = true;
      break;
    }



    int dim;

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



int GridGraph::calcTgtHeightCoord(float tgt_height)
{
  int coord = static_cast<int>(floorf( (tgt_height - _origin.z) / _gridstep + 0.5f));

  if (coord < 0)
    coord = 0;

  if (coord > _d - 1 )
    coord = _d -1;

  return coord;
}

