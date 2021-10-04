#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/utils/grid_graph.h>
#include <queue>
#include <cpc_motion_planning/ugv/evaluator/ugv_hybrid_evaluator.h>

class HybridDijkstra : public GridGraph
{
public:
  HybridDijkstra(int maxX, int maxY, int maxZ);
  void dijkstra_with_theta(CUDA_GEO::coord glb_tgt);
  void hybrid_dijkstra_with_int_theta(CUDA_GEO::coord glb_tgt);
  void hybrid_dijkstra_with_int_theta_with_line(CUDA_GEO::coord glb_tgt,CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b);
  void update_neighbour(const CUDA_GEO::coord &c, const int3 &shift, nodeInfo *m, float trans_cost);
  CUDA_GEO::coord hybrid_bfs(const float3 &start_pose, CUDA_GEO::coord glb_tgt);
  void hybrid_update_neighbour(const float3 &shift_pose, nodeInfo *m, float trans_cost);
  void hybrid_update_neighbour_with_line(const float3 &shift_pose, nodeInfo *m, float trans_cost,const CUDA_GEO::pos &seg_a, const CUDA_GEO::pos &seg_b);
  ~HybridDijkstra()
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

  struct shift_child
  {
    int3 shift;
    float3 shift_pose;
    float cost;
  };

  //---
  std::vector<float3> get_shortest_path(const float3 &in)
  {
    std::vector<float3> output;
    CUDA_GEO::coord s;
    s.x = floorf( (in.x - _origin.x) / _gridstep + 0.5f);
    s.y = floorf( (in.y - _origin.y) / _gridstep + 0.5f);
    s.z = theta2grid(in.z);

    //Find the next minimum coord
    nodeInfo* s_node = hybrid_getNode(s);
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
    delta_t = 0.4f;
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

private:
  std::vector<shift_child> children[THETA_GRID_SIZE];
  nodeInfo *m_id_map; // Identity map, store HybridDijkstra related params
  nodeInfo *m_init_id_map; // A copy for reset
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;

  int positive_modulo(int i, int n) const
  {
    return (i % n + n) % n;
  }


  nodeInfo* hybrid_getNode(CUDA_GEO::coord s)
  {
    if (s.x<0 || s.x>=_w ||
        s.y<0 || s.y>=_h ||
        s.z<0 || s.z>=THETA_GRID_SIZE)
      return nullptr;

    return &m_id_map[coord2index(s)];
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

    // while doing dijkstra, we start from the goal so very thing is growing backaward
    if (v > 0.1f)
      child.cost += fabsf(2.0f*v);

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

  float hybrid_obsCostAt(CUDA_GEO::coord s, float default_value) const
  {
    float cost = 0;
    float dist = getMinDist(s);
    cost += expf(-9.5f*dist)*10;
    if (dist < 0.41f)
      cost += 4000;
    return cost;
  }

  bool hybrid_is_free(CUDA_GEO::coord s) const
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

  float getMinDist(const CUDA_GEO::coord &s) const
  {
    // Calculate the equivilent theta
    float theta = grid2theta(s.z);

    // Calculate the center position
    CUDA_GEO::pos p;
    p.x = s.x * _gridstep + _origin.x;
    p.y = s.y * _gridstep + _origin.y;

    float2 c_f,c_r;
    calculate_bounding_centres(p, theta, c_r, c_f);

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

  CUDA_GEO::coord float32coord(const float3& pose)
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
};

#endif // DIJKSTRA_H
