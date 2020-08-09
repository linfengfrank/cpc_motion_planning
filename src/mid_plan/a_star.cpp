#include "mid_plan/a_star.h"
#include <ros/console.h>
#include <stack>
#include <chrono>
Astar::Astar(int maxX, int maxY, int maxZ):
  GridGraph(maxX,maxY,maxZ)
{

}

std::vector<CUDA_GEO::coord> Astar::AStar2D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone,  float &length,
                                            const CUDA_GEO::coord *crd_shift, SeenDist *last_val_map)
{
  std::vector<CUDA_GEO::coord> output;

  //First clear the old information
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
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

          float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy))*getGridStep() +
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
      length += sqrtf(static_cast<float>(shift.square()))*_gridstep;
    }
  }

  if (output.size() == 0)
  {
    ROS_ERROR("Path size is zero.");
    exit(-1);
  }

  return output;
}

std::vector<CUDA_GEO::coord> Astar::AStar3D(const CUDA_GEO::coord &goal, const CUDA_GEO::coord &start, bool reached_free_zone,  float &length,
                                            const CUDA_GEO::coord *crd_shift, SeenDist *last_val_map)
{
  std::vector<CUDA_GEO::coord> output;

  //First clear the old information
  memcpy(_id_map,_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*_d));
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

  auto start_time = std::chrono::steady_clock::now();
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
        for (int iz=-1; iz<=1;iz++)
        {
          if (ix==0 && iy ==0 && iz == 0)
            continue;

          child_coord.x = my_coord.x + ix;
          child_coord.y = my_coord.y + iy;
          child_coord.z = my_coord.z + iz;
          nodeInfo* p = getNode(child_coord);
          if (p && !p->inClosed)
          {

            float new_g = sqrtf(static_cast<float>(ix*ix+iy*iy+iz*iz))*getGridStep() +
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
    auto end_time = std::chrono::steady_clock::now();
    long passed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if(passed_time > 15)
      break;
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
      length += sqrtf(static_cast<float>(shift.square()))*_gridstep;
    }
  }

  if (output.size() == 0)
  {
    ROS_ERROR("Path size is zero.");
    exit(-1);
  }

  return output;
}

unsigned int Astar::findTargetCoord(const std::vector<CUDA_GEO::coord> &path)
{
  // If there is no path, return the center point
  if (path.size() == 0)
    return 0;

  unsigned int anchor = static_cast<unsigned int>(path.size() - 1);
  unsigned int target = 0;

  for (unsigned int i=0; i<path.size(); i++)
  {
    // find the largest distance
    unsigned int max_id = 0;
    float max_dist = 0;
    for (unsigned int j = target; j< anchor; j++)
    {
      float dist = point2lineDist(path[anchor],path[target],path[j],3);
      if (dist > max_dist)
      {
        max_dist = dist;
        max_id = j;
      }
    }

    if (max_dist*_gridstep > 0.5f)
    {
      target = max_id;
    }
    else
    {
      break;
    }
  }

  return target;
}

unsigned int Astar::findTargetCoordLos(std::vector<CUDA_GEO::coord> path, CUDA_GEO::coord start, unsigned int start_idx, float look_ahead_diff)
{
  // If there is no path, return the center point
  if (path.size() == 0)
    return 0;

  std::reverse(path.begin(),path.end());
  unsigned int target = 0;
  for (unsigned int i=start_idx; i<path.size(); i++)
  {
    if(isLOS(start,path[i],0.1f))
      target = i;
    else
      break;
  }

  unsigned int target_look_ahead = target;
  for (unsigned int i=target; i<path.size(); i++)
  {
    if(point2lineDist(start,path[i],path[target])*_gridstep > look_ahead_diff)
      break;
    else
      target_look_ahead = i;
  }

  return path.size()-target_look_ahead-1;
}

std::vector<CUDA_GEO::pos> Astar::findSplitCoords(const std::vector<CUDA_GEO::coord> &path)
{
  std::vector<CUDA_GEO::pos> split_pts;

  // If there is no path, return the center point
  if (path.size() == 0)
  {
    split_pts.push_back(coord2pos(CUDA_GEO::coord(_w/2,_h/2,_d/2)));
    return split_pts;
  }

  std::stack<std::pair<unsigned int,unsigned int>> pls; // fist is anchor, second is target
  pls.push(std::make_pair(path.size() - 1, 0));

  unsigned int check = static_cast<unsigned int>(path.size()) - 1;

  split_pts.push_back(coord2pos(path.back()));
  while (pls.size() > 0)
  {
    unsigned int target = pls.top().second;
    unsigned int anchor = pls.top().first;
    pls.pop();
    // find the largest distance
    unsigned int max_id = 0;
    float max_dist = 0;
    for (unsigned int j = target; j< anchor; j++)
    {
      float dist = point2lineDist(path[anchor],path[target],path[j]);
      if (dist > max_dist)
      {
        max_dist = dist;
        max_id = j;
      }
    }

    if (max_dist*_gridstep > 0.5f)
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
  return split_pts;
}

bool Astar::checkTopo(const std::vector<CUDA_GEO::coord> &path_a,const std::vector<CUDA_GEO::coord> &path_b)
{
  size_t K=20;
  for (size_t i=0; i<K; i++)
  {
    size_t idx_a = i*path_a.size()/K;
    size_t idx_b = i*path_b.size()/K;
    std::vector<CUDA_GEO::coord> line = rayCast(path_a[idx_a], path_b[idx_b]);
    for (CUDA_GEO::coord &c : line)
    {
      if(isOccupied(c,1.0f))
        return false;
    }
  }
  return true;
}
