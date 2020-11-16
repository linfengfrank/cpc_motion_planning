#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/grid_graph.h>
#include <queue>
#include <cpc_motion_planning/path.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_hybrid_evaluator.h>

class HybridAstar : public GridGraph
{
public:
  struct path_info
  {
    float3 pose;
    float3 action;
  };

  struct Transition
  {
    CUDA_GEO::coord shift_children;
    float3 action;
    float cost;
  };

  HybridAstar(int maxX, int maxY, int maxZ);

  std::vector<path_info> plan(const float3 &start_pose, CUDA_GEO::coord glb_tgt);
  cpc_motion_planning::path split_path(std::vector<path_info> raw_path);
  std::vector<size_t> split_merge(const std::vector<path_info> &path);
  void split_forward_backward_driving(cpc_motion_planning::path &cell, const std::vector<path_info> &path);

  ~HybridAstar()
  {
    delete [] m_id_map;
    delete [] m_init_id_map;
  }


  float hybrid_getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const
  {
    if (s.x<0 || s.x>=_w || s.y<0 || s.y>=_h || s.z<0 || s.z>=THETA_GRID_SIZE)
    {
      return default_value;
    }
    else
    {
      return m_id_map[coord2index(s)].g;
    }
  }

  //---
  std::vector<float3> get_shortest_path(const float3 &in)
  {
    std::vector<float3> output;
    CUDA_GEO::coord s;
    s.x = floorf( (in.x - _origin.x) / _gridstep + 0.5f);
    s.y = floorf( (in.y - _origin.y) / _gridstep + 0.5f);
    s.z = theta2grid(in.z);

    //Find the next minimum coord
    nodeInfo* s_node = m_getNode(s);
    while(1)
    {
      if(s_node)
      {
        output.push_back(coord2float3(s_node->c));
        s_node = s_node->ptr2parent;
      }
      else
      {
        break;
      }
    }

    return output;
  }
  //---
  void set_neighbours()
  {
    // First set the angle discretization
    for (int i=0; i<THETA_GRID_SIZE; i++)
    {
      int2 dxdy = idx2dxdy(NB_SQ_W,i);
      m_theta_grid[i] = atan2(dxdy.y, dxdy.x);
      //std::cout<<"Theta: "<<m_theta_grid[i]<<std::endl;
    }

    // Populate the transition vector
    for (int i=0; i<THETA_GRID_SIZE; i++)
    {
      add_trans_rotation(i,m_theta_based_trans[i]);
      add_trans_straight_move(i,m_theta_based_trans[i]);

//      std::cout<<i<<": "<<m_theta_based_trans[i].size()<<std::endl;
//      for (int j=0;j<m_theta_based_trans[i].size();j++)
//      {
//        Transition tr = m_theta_based_trans[i][j];
//        std::cout<<"theta: "<<i<<", "<<m_theta_grid[i]<<std::endl;
//        std::cout<<"int3: "<<tr.shift_children.x<<" "<<tr.shift_children.y<<" "<<tr.shift_children.z<<std::endl;
//        std::cout<<"action: "<<tr.action.x<<" "<<tr.action.y<<" "<<tr.action.z<<std::endl;
//        std::cout<<"cost: "<<tr.cost<<std::endl;
//        std::cout<<"-------------------"<<std::endl;
//      }
    }
  }
  //---
  bool hybrid_isfree(const float3& pose) const
  {
    return hybrid_isfree(float32coord(pose));
  }
private:
  std::vector<Transition> m_theta_based_trans[THETA_GRID_SIZE];
  float m_theta_grid[THETA_GRID_SIZE];
  nodeInfo *m_id_map; // Identity map, store HybridAstar related params
  nodeInfo *m_init_id_map; // A copy for reset
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;

  int positive_modulo(int i, int n) const
  {
    return (i % n + n) % n;
  }


  nodeInfo* m_getNode(CUDA_GEO::coord s)
  {
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=THETA_GRID_SIZE)
      return nullptr;

    return &m_id_map[coord2index(s)];
  }

  int2 idx2dxdy(int w, int idx)
  {
    int2 output;
    int hw = w/2;
    if (idx >=0 && idx <= hw)
    {
        output.x = hw;
        output.y = idx;
    }
    else if (idx >= hw+1 && idx <= hw + w -1)
    {
        output.x = 2*hw - idx;
        output.y = hw;
    }
    else if (idx>=hw+w && idx<=hw+2*w-2)
    {
        output.x = hw-w+1;
        output.y = 2*hw+w-1-idx;
    }
    else if (idx>=hw+2*w-1 && idx<=hw+3*w-3)
    {
        output.x = idx-3*w+3;
        output.y = hw-w+1;
    }
    else if (idx>=hw+3*w-2 && idx <=2*hw+3*w-4)
    {
        output.x = hw;
        output.y = idx -4*w +4;
    }
    return output;
  }

  float grid2theta(int grid) const
  {
    grid = positive_modulo(grid,THETA_GRID_SIZE);
    return m_theta_grid[grid];
  }

  int theta2grid(float theta) const
  {
    float diff_min = 1e6;
    int diff_min_id = 0;
    float diff;
    for (int i = 0; i<THETA_GRID_SIZE; i++)
    {
      diff = fabsf(in_pi(theta-m_theta_grid[i]));

      if (diff < diff_min)
      {
          diff_min = diff;
          diff_min_id = i;
      }
    }
    return diff_min_id;
  }

  int2 find_shortest_int_nb(int w, const int2 &dxdy) const
  {
    int2 xy = dxdy;
    int hw = w/2;

    for (int b = hw; b>=1; b--)
    {
      if(dxdy.x % b==0 && dxdy.y % b==0)
      {
        xy.x = dxdy.x/b;
        xy.y = dxdy.y/b;
        break;
      }
    }
    return xy;
  }

  void add_trans_rotation(int i, std::vector<Transition>& trans_vec)
  {
    float omega = 0.3f;
    float theta = m_theta_grid[i];
    Transition trans;

    // + rotation
    float theta_plus = m_theta_grid[positive_modulo(i+1,THETA_GRID_SIZE)];
    trans.shift_children = CUDA_GEO::coord(0,0,1);
    trans.action = make_float3(omega, 0, fabsf(in_pi(theta_plus - theta)/omega));
    trans.cost = trans.action.z;
    trans_vec.push_back(trans);

    // - rotation
    float theta_minu = m_theta_grid[positive_modulo(i-1,THETA_GRID_SIZE)];
    trans.shift_children = CUDA_GEO::coord(0,0,-1);
    trans.action = make_float3(-omega, 0, fabsf(in_pi(theta_minu - theta)/omega));
    trans.cost = trans.action.z;
    trans_vec.push_back(trans);
  }

  void add_trans_straight_move(int i, std::vector<Transition>& trans_vec)
  {
    Transition trans;
    float vel = 0.5f;
    float dist;

    // forward
    int2 dxdy_forward = idx2dxdy(NB_SQ_W,i);
    int2 xy_forward = find_shortest_int_nb(NB_SQ_W,dxdy_forward);
    dist = getGridStep()*sqrtf(dot(xy_forward,xy_forward));
    trans.shift_children = CUDA_GEO::coord(xy_forward.x,xy_forward.y,0);
    trans.action = make_float3(0, vel, dist/vel);
    trans.cost = trans.action.z;
    trans_vec.push_back(trans);

    // backward
    int2 dxdy_backward = idx2dxdy(NB_SQ_W,positive_modulo(i+2*NB_SQ_W-2,THETA_GRID_SIZE));
    int2 xy_backward = find_shortest_int_nb(NB_SQ_W,dxdy_backward);
    dist = getGridStep()*sqrtf(dot(xy_backward,xy_backward));
    trans.shift_children = CUDA_GEO::coord(xy_backward.x,xy_backward.y,0);
    trans.action = make_float3(0, -vel, dist/vel);
    trans.cost = trans.action.z;
    trans_vec.push_back(trans);
  }



  float hybrid_obsCostAt(const CUDA_GEO::coord &s, float default_value) const
  {
    float cost = 0;
    float dist = getMinDist(s);
    cost += expf(-9.5f*dist)*150;
    if (dist < 0.41f)
      cost += 1000;
    return cost;
  }

  bool hybrid_isfree(const CUDA_GEO::coord &s) const
  {
    float dist = getMinDist(s);

    if (dist < 0.41f)
      return false;
    else
      return true;
  }

  float getEDT(const float2 &p) const
  {
    CUDA_GEO::coord s;
    s.x = floorf( (p.x - _origin.x) / _gridstep + 0.5f);
    s.y = floorf( (p.y - _origin.y) / _gridstep + 0.5f);
    s.z = 0;
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=_d)
    {
      return 0;
    }
    else
    {
      return _val_map[coord2index(s)].d * _gridstep;
    }
  }

  void calculate_bounding_centres(const CUDA_GEO::coord &s, float2 &c_r, float2 &c_f) const
  {
    float theta = grid2theta(s.z);
    float2 p;
    p.x = s.x * _gridstep + _origin.x;
    p.y = s.y * _gridstep + _origin.y;
    float2 uni_dir = make_float2(cosf(theta),sinf(theta));
    c_f = p + 0.25f*uni_dir;
    c_r = p - 0.25f*uni_dir;
  }

  float getMinDist(const CUDA_GEO::coord &s) const
  {
    float2 c_f,c_r;
    calculate_bounding_centres(s, c_r, c_f);
    return min(getEDT(c_r),getEDT(c_f));
  }

  float3 coord2float3(const CUDA_GEO::coord &s)
  {
    float3 output;
    output.x = s.x * _gridstep + _origin.x;
    output.y = s.y * _gridstep + _origin.y;
    output.z = grid2theta(s.z);
    return output;
  }

  CUDA_GEO::coord float32coord(const float3& pose) const
  {
    CUDA_GEO::coord s;
    s.x = floorf( (pose.x - _origin.x) / _gridstep + 0.5f);
    s.y = floorf( (pose.y - _origin.y) / _gridstep + 0.5f);
    s.z = theta2grid(pose.z);
    return s;
  }

  float dist2D(const CUDA_GEO::coord & c1, const CUDA_GEO::coord & c2)
  {
    return sqrtf((c1.x-c2.x)*(c1.x-c2.x) + (c1.y-c2.y)*(c1.y-c2.y))*getGridStep();
  }

  float in_pi(float in) const
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  inline float pnt2line_dist(const float3 & c1, const float3 & c2, const float3 & c0, float scale_verticle = 1.0f)
  {
    float3 a = c1-c0;
    float3 b = c2-c1;

    a.z = a.z * scale_verticle;
    b.z = b.z * scale_verticle;

    float a_square = dot(a,a);
    float b_square = dot(b,b);
    float a_dot_b = a.x*b.x + a.y*b.y + a.z*b.z;

    if (b_square < 1e-3)
      return sqrtf(static_cast<float>(a_square));

    return sqrtf(static_cast<float>(a_square*b_square - a_dot_b*a_dot_b)/static_cast<float>(b_square));
  }

  std::vector<path_info> select_between_idx(const std::vector<path_info> &path, size_t start, size_t end)
  {
    std::vector<path_info> output;
    for (size_t i=start; i<=end; i++)
    {
      output.push_back(path[i]);
    }
    return output;
  }

  unsigned char determine_action_case(const float3 &action)
  {
    // 0 -> done nothing
    // 1 -> rotate
    // 2 -> forward
    // 3 -> backward

    float w = action.x;
    float v = action.y;
    float dt = action.z;

    if (fabsf(w*dt) > 0.01f)
        return 1;


    if (v*dt > 0.01f)
        return 2;

    if (v*dt < -0.01f)
        return 3;

    return 0;
  }

  cpc_motion_planning::path_action construct_path_action(const std::vector<path_info> &path, unsigned char t)
  {
    cpc_motion_planning::path_action output;
    output.type = t;
    for (size_t i=0; i<path.size(); i++)
    {
      output.x.push_back(path[i].pose.x);
      output.y.push_back(path[i].pose.y);
      output.theta.push_back(path[i].pose.z);
    }
    return output;
  }

  float3 angle_dev(const std::vector<path_info> &path, size_t idx_a, size_t idx_b)
  {
    float3 output; //x for dev, y for angle_diff, z for horizontal distance
    float3 delta = path[idx_a].pose - path[idx_b].pose;

    float horizon_dist = sqrtf(delta.x*delta.x + delta.y*delta.y);
    float angle_diff = fabsf(delta.z);

    float dev;
    if (horizon_dist > 1e-3)
      dev = angle_diff / horizon_dist;
    else
      dev = 1000.0f;

    output.x = dev;
    output.y = angle_diff;
    output.z = horizon_dist;

    return  output;
  }
};

#endif // DIJKSTRA_H
