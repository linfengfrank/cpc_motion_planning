#ifndef UGV_DP_CONTROL_H
#define UGV_DP_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <vector>
namespace UGV
{
class UGVDPControl
{
public:
  UGVDPControl()
  {

  }

  ~UGVDPControl()
  {

  }


  void load_data(const std::string& file_location, CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    factory.load_cuda_matrix<4,UGVModel::Input>(&S_A,file_location+"SA.dat",load_to_host);
    factory.load_uniform_bin(file_location+"pos_bin.dat",ubc.bins[0]);
    factory.load_uniform_bin(file_location+"vel_bin.dat",ubc.bins[1]);
    factory.load_uniform_bin(file_location+"theta_bin.dat",ubc.bins[2]);
    factory.load_uniform_bin(file_location+"w_bin.dat",ubc.bins[3]);
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {
    factory.free_cuda_matrix<4,UGVModel::Input>(S_A, load_from_host);
  }

  __host__ __device__
  float3 dp_control(const UGVModel::State &s, const float3 &site) const
  {
    // Construct the relative state
    float s_relative[4];
    s_relative[0] = s.s - site.x; // relative station
    s_relative[1] = s.v; // relative velocity
    s_relative[2] = s.theta - site.y; // relative angle
    s_relative[3] = s.w; //relative angular speed

    UGVModel::Input u = CUDA_MAT::get_control_uniform_bin_4(s_relative, *S_A, ubc);
    return make_float3(u.acc,u.alpha,0.0f);
  }

  __host__ __device__
  float3 update_site_target(const UGVModel::State &s, const float3 &site)
  {
    float3 output = site;
    output.x += s.s;
    output.y += s.theta;
    return output;
  }

  __host__ __device__
  void update_collision_state(PSO::CollisionState &collision_state, float min_dist, float initial_dist, float &first_collision_time, float t)
  {
    switch (collision_state)
    {
    case PSO::FREE:
    {
      if (min_dist < PSO::MIN_DIST)
      {
        collision_state = PSO::COLLISION;
        first_collision_time = t;
      }
      break;
    }

    case PSO::COLLISION:
    {
      // Do nothing
      break;
    }

    case PSO::INIT_COLLISION:
    {
      if (min_dist < PSO::MIN_DIST)
      {
        // Does not allow the situation to get worse
        if (min_dist < initial_dist || min_dist < 0.21f)
        {
          collision_state = PSO::COLLISION;
          first_collision_time = 0;
        }
        // Other wise do nothing, have not get out of collision yet
      }
      else
      {
        // Got out of collision, go to the free state
        collision_state = PSO::FREE;
      }
      break;
    }
    }
  }

  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr, bool &is_traj_collision)
  {
    typename Model::State s = m.get_ini_state();

    float cost = 0;
    float dt = PSO::PSO_SIM_DT;

    PSO::EvaData data; //Data to be received from the evaluator
    data.current_v = s.v;
    eva.process_cost(s,map,0,data); // Conduct measurement of the initial state
    PSO::CollisionState collision_state; //Collision state
    float first_collision_time = 1000; //Time of first collision
    float initial_dist = data.min_dist; //Initial distance to obstacle
    float traj_min_dist = initial_dist; //The minimum distance on the trajectory

    // Init the collision state depends on the initial situation
    if (data.min_dist < PSO::MIN_DIST)
      collision_state = PSO::INIT_COLLISION;
    else
      collision_state = PSO::FREE;

    int prev_i = -1;
    float3 site_target;
    for (float t=0.0f; t<sw.total_t; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i!=prev_i || prev_i == -1)
      {
        //update the site target
        site_target = update_site_target(s,ttr[i]);
        data.is_forward = (ttr[i].z > 0 ? true : false);
      }
      prev_i = i;

      float3 u = dp_control(s, site_target);
      m.model_forward(s,u,dt);

      //cost += 0.1f*sqrtf(u.x*u.x + 0.5f*u.y*u.y + u.z*u.z);
      cost += eva.process_cost(s,map,t,data);

      update_collision_state(collision_state, data.min_dist, initial_dist, first_collision_time, t);

      if (data.min_dist < traj_min_dist)
        traj_min_dist = data.min_dist;
    }

    // If there is a collision in close time, OR the vehicle has not be able to
    // recover from a initial collision, OR the minimum distance on the trajectory
    // is lower than 0.2, the trajectory is deemed as collision.
    if ((collision_state == PSO::COLLISION && first_collision_time < 0.31f) || collision_state == PSO::INIT_COLLISION || traj_min_dist < 0.21f)
      is_traj_collision = true;
    else
      is_traj_collision = false;

    cost += eva.final_cost(s,map);
    return cost;
  }

  template<class Model, class Swarm>
  __host__ __device__
  std::vector<typename Model::State> generate_trajectory(Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    std::vector<typename Model::State> traj;
    typename Model::State s = m.get_ini_state();
    float dt = PSO::PSO_CTRL_DT;
    int prev_i = -1;
    float3 site_target;
    for (float t=0.0f; t<sw.total_t; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i!=prev_i || prev_i == -1)
      {
        //update the site target
        site_target = update_site_target(s,ttr[i]);
      }
      prev_i = i;

      float3 u = dp_control(s, site_target);
      m.model_forward(s,u,dt);
      traj.push_back(s);
    }
   return traj;
  }

  CUDA_MAT::Matrix<4,UGVModel::Input> *S_A;

  UniformBinCarrier ubc;
};
}
#endif // UGV_DP_CONTROL_H
