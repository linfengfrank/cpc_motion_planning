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

  struct shift_child
  {
    int3 shift;
    float3 shift_pose;
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
    std::vector<float3> mps;
    float delta_t = 0;
    delta_t = 1.0f;
    add_motion_rotation(mps, 2.0f*M_PI/(delta_t*static_cast<float>(THETA_GRID_SIZE)), delta_t);
    delta_t = 1.0f;
    add_motion_straight_move(mps, 1.01f*getGridStep()/delta_t, delta_t);

    for (size_t i = 0; i< THETA_GRID_SIZE; i++)
    {
      float theta = grid2theta(static_cast<int>(i));

      for (float3 mp : mps)
      {
        float v = mp.y;
        float w = mp.x;
        float dt = mp.z;
        shift_child child = get_shift_child(theta,w,v,dt);

        if (child.shift.x == 0 && child.shift.y == 0 && child.shift.z == 0)
          continue;

        children[i].push_back(child);
      }
    }
  }
  //---
  bool hybrid_isfree(const float3& pose) const
  {
    return hybrid_isfree(float32coord(pose));
  }
private:
  std::vector<shift_child> children[THETA_GRID_SIZE];
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

  float m_obsCostAt(CUDA_GEO::coord s, float default_value, bool &occupied)
  {
    s.z = 0;
    return obsCostAt(s,default_value,occupied);
  }

  float grid2theta(int grid) const
  {
    grid = positive_modulo(grid,THETA_GRID_SIZE);
    return 2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)*static_cast<float>(grid);
  }

  int theta2grid(float theta) const
  {
    int grid = floor(theta/(2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)) + 0.5);
    return positive_modulo(grid,THETA_GRID_SIZE);
  }

  int float2grid(float p) const
  {
    return floorf( p / getGridStep() + 0.5f);
  }

  int theta2grid_raw(float theta) const
  {
    return floor(theta/(2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)) + 0.5);
  }

  shift_child get_shift_child(float theta, float w, float v, float dt)
  {
    shift_child child;
    float dx,dy,dxp,dyp;
    float ct = cosf(theta);
    float st = sinf(theta);
    if (fabsf(w) < 1e-3)
    {
      dxp = v*dt;
      dyp = 0;
    }
    else
    {
      float R = v/w;
      dxp = R*sinf(w*dt);
      dyp = R*(1-cosf(w*dt));
    }

    dx = ct*dxp - st*dyp;
    dy = st*dxp + ct*dyp;
    child.shift.x = float2grid(dx);
    child.shift.y = float2grid(dy);
    child.shift.z = theta2grid_raw(w*dt);

    child.shift_pose.x = dx;
    child.shift_pose.y = dy;
    child.shift_pose.z = w*dt;

    child.cost = 0.2f*fabsf(dt);

    child.action = make_float3(w,v,dt);

    return child;
  }

  void add_motion_rotation(std::vector<float3> &mps, float w, float t)
  {
    mps.push_back(make_float3(w,0,t));
    mps.push_back(make_float3(-w,0,t));
  }

  void add_motion_straight_move(std::vector<float3> &mps, float v, float t)
  {
    mps.push_back(make_float3(0,v,t));
    mps.push_back(make_float3(0,-v,t));
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

  float in_pi(float in)
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
