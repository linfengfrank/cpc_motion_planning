#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/ugv/controller/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/controller/ugv_jlt_control.h>
#include <cpc_motion_planning/ugv/swarm/ugv_swarm.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_nf1_evaluator.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_hybrid_evaluator.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_corridor_evaluator.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_ref_traj_evaluator.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_recover_evaluator.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include "cublas_v2.h"
namespace PSO
{
template<class Swarm>
void setup_random_states(const Swarm &sw);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void initialize_particles(bool first_run,
                          const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void iterate_particles(float weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//--------
template<class Model, class Controller, class Evaluator, class Swarm>
void iterate_particles_de(const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//---------
template<class Swarm>
void copy_best_values(float *best_values, const Swarm &sw);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void path_integral(const EDTMap &map, const Evaluator &eva,
                   const Model &m, const Controller &ctrl,
                   const Swarm &sw, const typename Swarm::Trace &tr);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
float evaluate_trace(const EDTMap &map, const Evaluator &eva,
                   const Model &m, const Controller &ctrl,
                   const Swarm &sw, const typename Swarm::Trace &tr);
}
#endif // PSO_KERNELS_CUH
