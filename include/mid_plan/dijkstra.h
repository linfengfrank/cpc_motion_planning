#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <mid_plan/grid_graph.h>
#include <queue>

#define THETA_GRID_SIZE 16
class Dijkstra : public GridGraph
{
public:
  Dijkstra(int maxX, int maxY, int maxZ);
  void dijkstra2D(CUDA_GEO::coord glb_tgt);
  void dijkstra2D_with_line(CUDA_GEO::coord glb_tgt, CUDA_GEO::pos seg_a, CUDA_GEO::pos seg_b);
  void bfs2D(CUDA_GEO::coord glb_tgt);
  void dijkstra3D(CUDA_GEO::coord glb_tgt);
  void dijkstra_with_theta(CUDA_GEO::coord glb_tgt);
  void update_neighbour(const CUDA_GEO::coord &c, const int3 &shift, const float &m_g, float trans_cost);
  ~Dijkstra()
  {
    delete [] m_id_map;
    delete [] m_init_id_map;
  }


  float m_getCost2Come(const CUDA_GEO::coord & s, const float &default_value) const
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
    float cost;

    bool operator<(const shift_child& rhs) const
    {
      if(shift.x < rhs.shift.x)
      {
        return true;
      }
      else if (shift.x == rhs.shift.x && shift.y < rhs.shift.y)
      {
        return true;
      }
      else if (shift.x == rhs.shift.x && shift.y == rhs.shift.y && shift.z < rhs.shift.z)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  };

private:
  std::set<shift_child> children[THETA_GRID_SIZE];
//  int3 children[THETA_GRID_SIZE][4];
  nodeInfo *m_id_map; // Identity map, store Dijkstra related params
  nodeInfo *m_init_id_map; // A copy for reset
  std::queue<nodeInfo*> _Q;
  SortedSet<nodeInfo*> _OQ;

  int positive_modulo(int i, int n)
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

  float grid2theta(int grid)
  {
    grid = positive_modulo(grid,THETA_GRID_SIZE);
    return 2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)*static_cast<float>(grid);
  }

  int theta2grid(float theta)
  {
    int grid = floor(theta/(2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)) + 0.5);
    return positive_modulo(grid,THETA_GRID_SIZE);
  }

  int float2grid(float p)
  {
    return floorf( p / getGridStep() + 0.5f);
  }

  int theta2grid_raw(float theta)
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
      child.shift.z = 0;
      dxp = v*dt;
      dyp = 0;
      dx = ct*dxp - st*dyp;
      dy = st*dxp + ct*dyp;
      child.shift.x = float2grid(dx);
      child.shift.y = float2grid(dy);
      //child.cost = fabsf(v*dt);
    }
    else
    {
      child.shift.z = theta2grid_raw(w*dt);
      float R = v/w;
      dxp = R*sinf(w*dt);
      dyp = R*(1-cosf(w*dt));
      dx = ct*dxp - st*dyp;
      dy = st*dxp + ct*dyp;
      child.shift.x = float2grid(dx);
      child.shift.y = float2grid(dy);
      //child.cost = fabsf(R*w*dt);// + 2*fabsf(w*dt);
    }
    child.cost = 0.5f*fabsf(dt);

    // while doing dijkstra, we start from the goal so very thing is growing backaward
    if (v > 0.1f)
      child.cost += fabsf(2.0f*v);

    return child;
  }

  float mm_obsCostAt(CUDA_GEO::coord s, float default_value) const
  {
    SeenDist* map_ptr;
    map_ptr = _val_map;
    s.z = 0;

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

 //   cost += expf(-7.0f*dist)*40;
    if (dist < 0.61f)
      cost += 100;

    if (dist <= 0.71f)
    {
      cost += (150.0f-210.0f*dist);
    }
    else if (dist < 1.5f)
    {
      cost += (3.75f-2.5f*dist);
    }


    return cost;
  }
};

#endif // DIJKSTRA_H
