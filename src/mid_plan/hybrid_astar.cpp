#include "mid_plan/hybrid_astar.h"
#include <ros/console.h>
#include <stack>

HybridAstar::HybridAstar(int maxX, int maxY, int maxZ):
  GridGraph(maxX,maxY,maxZ)
{
  m_id_map = new nodeInfo[_w*_h*THETA_GRID_SIZE];
  m_init_id_map = new nodeInfo[_w*_h*THETA_GRID_SIZE];

  //Assign coords value for all members in the id map
  CUDA_GEO::coord s;
  for (s.x=0; s.x<_w; s.x++)
  {
    for (s.y=0; s.y<_h; s.y++)
    {
      for (s.z=0; s.z<THETA_GRID_SIZE; s.z++)
      {
        m_init_id_map[coord2index(s)].c=s;
      }
    }
  }
}

std::vector<HybridAstar::path_info> HybridAstar::plan(const float3 &start_pose, CUDA_GEO::coord glb_tgt)
{
  std::vector<path_info> path;

  //First clear the old information
  memcpy(m_id_map,m_init_id_map,sizeof(nodeInfo)*static_cast<size_t>(_w*_h*THETA_GRID_SIZE));
  _PQ.clear();

  // Useful variables
  CUDA_GEO::coord closest_coord,start;
  float min_h = std::numeric_limits<float>::infinity();

  //Set the root
  start = float32coord(start_pose);
  nodeInfo* ni=m_getNode(start);
  if (ni)
  {
    ni->g = 0;
    ni->h = dist2D(ni->c,glb_tgt);
    ni->pose = coord2float3(start);
    _PQ.insert(ni,ni->g);
    closest_coord = ni->c;
    min_h = ni->h;
  }
  else
  {
    ROS_ERROR("First point outside the boundary.");
    exit(-1);
  }

  // start the hybrid D
  bool reached_free_zone = false;
  while (_PQ.size()>0)
  {
    ni=_PQ.pop();
    ni->inClosed = true;

    if(hybrid_isfree(ni->c))
      reached_free_zone = true;

    if (min_h > ni->h)
    {
      min_h = ni->h;
      closest_coord = ni->c;
    }

    for (shift_child child : children[positive_modulo(ni->c.z,THETA_GRID_SIZE)])
    {

      float3 child_pose = child.shift_pose + ni->pose;
      CUDA_GEO::coord pc = float32coord(child_pose);
      nodeInfo* p = m_getNode(pc);

      if (p && !p->inClosed)
      {
        float new_g = child.cost + ni->g + hybrid_obsCostAt(pc,0);
        if (p->g > new_g && !(!hybrid_isfree(pc) && reached_free_zone))
        {
          p->g = new_g;
          p->pose = child_pose;
          p->ptr2parent = ni;
          p->action_lead2me = child.action;
          p->h = dist2D(p->c,glb_tgt);
          _PQ.insert(p,p->g);
        }
      }
    }
  }

  ni=m_getNode(closest_coord);
  std::vector<nodeInfo*> node_list;
  while (ni != nullptr)
  {
    node_list.push_back(ni);
    ni = ni->ptr2parent;
  }
  std::reverse(node_list.begin(),node_list.end());

  path_info path_node;
  for (size_t i = 0; i < node_list.size(); i++)
  {
    path_node.pose = node_list[i]->pose;//coord2float3(node_list[i]->c);

    if (i+1 < node_list.size())
      path_node.action = node_list[i+1]->action_lead2me;
    else
      path_node.action = make_float3(0,0,0);

    path.push_back(path_node);
  }

  return path;
}

cpc_motion_planning::path HybridAstar::split_path(std::vector<path_info> path)
{
  cpc_motion_planning::path cell;

  // First un in pi all the angles
  for (size_t i=0; i<path.size(); i++)
  {
    if (i == 0)
      path[i].pose.z = in_pi(path[i].pose.z);
    else
      path[i].pose.z = path[i-1].pose.z + in_pi(path[i].pose.z - path[i-1].pose.z);
  }

  // Split and merege
  std::vector<size_t> split_idx = split_merge(path);


  size_t seg_start_idx = 0;
  if (split_idx.size() >=1 )
  {
    for (size_t i = 0;i<split_idx.size()-1;i++)
    {
      float3 dev_info = angle_dev(path,split_idx[i],split_idx[i+1]);

      if (dev_info.x >3.5f && dev_info.y > 3.1415f*0.2f)
      {
        // detected a turning part

        // add moving part
        split_forward_backward_driving(cell,select_between_idx(path,seg_start_idx,split_idx[i]));

        // add turning part
        cell.actions.push_back(construct_path_action(select_between_idx(path,split_idx[i],split_idx[i+1]), 1));

        seg_start_idx = split_idx[i+1];
      }
    }
  }

  if (path.size()>=1 && seg_start_idx < path.size() - 1)
  {
    split_forward_backward_driving(cell,select_between_idx(path,seg_start_idx,path.size() - 1));
  }



  return cell;
}

std::vector<size_t> HybridAstar::split_merge(const std::vector<path_info> &path)
{
  std::vector<size_t> split_idx;

  // If there is no path, return the center point
  if (path.size() <= 1)
  {
    return split_idx;
  }

  std::stack<std::pair<unsigned int,unsigned int>> task; // fist is anchor, second is target
  task.push(std::make_pair(path.size() - 1, 0));

  unsigned int check = static_cast<unsigned int>(path.size()) - 1;

  split_idx.push_back(path.size() - 1);
  while (task.size() > 0)
  {
    unsigned int target = task.top().second;
    unsigned int anchor = task.top().first;
    task.pop();
    // find the largest distance
    unsigned int max_id = 0;
    float max_dist = 0;
    for (unsigned int j = target; j< anchor; j++)
    {
      float dist = pnt2line_dist(path[anchor].pose,path[target].pose,path[j].pose);
      if (dist > max_dist)
      {
        max_dist = dist;
        max_id = j;
      }
    }

    if (max_dist > 0.2f)
    {
      task.push(std::make_pair(max_id, target));
      task.push(std::make_pair(anchor, max_id));
      target = max_id;
    }
    else
    {
      split_idx.push_back(target);
      if (target >= check)
      {
        std::cout<<"path size: "<<path.size()<<std::endl;
        ROS_ERROR("Wrong split sequence");
        exit(-1);
      }
      check = target;
    }
  }
  std::reverse(split_idx.begin(),split_idx.end());
  return split_idx;
}

void HybridAstar::split_forward_backward_driving(cpc_motion_planning::path &cell, const std::vector<path_info> &path)
{
  size_t seg_start_idx = 0;
  unsigned char moving_dir = 0;

  for (size_t i=0; i<path.size(); i++)
  {
    unsigned char dir = determine_action_case(path[i].action);
    if (moving_dir == 0)
    {
      if (dir >= 2)
        moving_dir = dir;
    }
    else
    {
      if (dir >= 2 && moving_dir != dir)
      {
        //detected a change of velocity sign
        cell.actions.push_back(construct_path_action(select_between_idx(path,seg_start_idx,i), moving_dir));
        seg_start_idx = i;
        moving_dir = dir;
      }
    }
  }

  if(seg_start_idx < path.size()-1)
  {
    cell.actions.push_back(construct_path_action(select_between_idx(path,seg_start_idx,path.size()-1), moving_dir));
  }
}

