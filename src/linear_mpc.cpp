#include "mpc/linear_mpc.h"

linear_mpc::linear_mpc(double dt, int N, double v_lim, double w_lim, double a_lim, double alpha_lim)
{
  m_dt = dt;
  m_initial_condition_set = false;
  m_reference_set = false;

  m_N = N;
  m_dim = 5*N+3;
  m_solver.data()->setNumberOfVariables(m_dim);
  m_solver.data()->setNumberOfConstraints(m_dim + 2*m_N);
  m_solver.settings()->setVerbosity(false);
  m_solver.settings()->setWarmStart(true);

  m_v_lim = v_lim;
  m_w_lim = w_lim;
  m_a_lim = a_lim;
  m_alpha_lim = alpha_lim;
}

void linear_mpc::set_cost(const Eigen::Matrix<double,3,3> &Q, const Eigen::Matrix<double,2,2> &R, const Eigen::Matrix<double,2,2> &S)
{
  m_Q = Q;
  m_R = R;
  m_S = S;

  m_H = Eigen::MatrixXd::Constant(m_dim, m_dim, 0);

  for(int i=0;i<=m_N;i++)
  {
    m_H.block(3*i,3*i,3,3)=Q;
  }

  for(int i=m_N+1;i<=2*m_N;i++)
  {
    m_H.block(m_N+1+2*i, m_N+1+2*i, 2, 2) = R;
  }

  Eigen::MatrixXd H_delta = Eigen::MatrixXd::Constant(2*m_N, 2*m_N, 0);
  for (int i=0; i<2*m_N-2; i++)
  {
    if(i%2 == 0)
    {
      //for v
      H_delta(i,i) = 2.0*S(0,0);
      H_delta(i,i+2) = -2.0*S(0,0);
    }
    else
    {
      //for w
      H_delta(i,i) = 2.0*S(1,1);
      H_delta(i,i+2) = -2.0*S(1,1);
    }
  }

  H_delta(2*m_N-2, 2*m_N-2) = S(0,0);
  H_delta(2*m_N-1, 2*m_N-1) = S(1,1);

  m_H.block(3*m_N+3, 3*m_N+3, 2*m_N, 2*m_N) += (H_delta.transpose() + H_delta)/2.0;
  m_gradient = Eigen::VectorXd::Constant(m_dim,1,0);
  m_A_con = Eigen::MatrixXd::Constant(m_dim+2*m_N, m_dim, 0);
  m_b_lb =  Eigen::VectorXd::Constant(m_dim+2*m_N, 1,     0);
  m_b_ub =  Eigen::VectorXd::Constant(m_dim+2*m_N, 1,     0);

  //set the invariant part int m_A_con and m_b_lb and m_b_ub
  int size_x = 3*m_N + 3;
  for (int i=0; i<m_N-1; i++)
  {
    m_A_con(m_dim + 2*i,     size_x + 2*i)     = -1;
    m_A_con(m_dim + 2*i,     size_x + 2*i + 2) =  1;
    m_A_con(m_dim + 2*i + 1, size_x + 2*i + 1) = -1;
    m_A_con(m_dim + 2*i + 1, size_x + 2*i + 3) =  1;

    m_b_ub(m_dim + 2*i)     = m_a_lim * m_dt;
    m_b_ub(m_dim + 2*i + 1) = m_alpha_lim * m_dt;

    m_b_lb(m_dim + 2*i)     = -m_a_lim * m_dt;
    m_b_lb(m_dim + 2*i + 1) = -m_alpha_lim * m_dt;
  }
  m_A_con(m_dim + 2*m_N - 2, size_x) = 1;
  m_A_con(m_dim + 2*m_N - 1, size_x + 1) = 1;
}


void linear_mpc::set_reference_and_state(const std::vector<double> &x_r, const std::vector<double> &y_r,
                   const std::vector<double> &th_r,const std::vector<double> &v_r,
                   const std::vector<double> &w_r, double x, double y, double th,
                                         double v_last, double w_last)
{
  m_reference_set = true;
  m_x_r = x_r;
  m_y_r = y_r;
  m_th_r= th_r;
  m_v_r = v_r;
  m_w_r = w_r;
  m_v_last = v_last;
  m_w_last = w_last;

  un_in_pi_ref(th);
  generate_ltv_model();
  generate_constraint_matrices();
  set_state(x,y,th);
}

void linear_mpc::generate_ltv_model()
{
  m_A_ltv.clear();
  m_B_ltv.clear();
  m_C_ltv.clear();

  Eigen::Matrix<double,3,3> A_k;
  Eigen::Matrix<double,3,2> B_k;
  Eigen::Matrix<double,3,1> C_k;
  for(int k=0; k<m_N; k++)
  {
    A_k<<1, 0, -m_v_r[k]*sin(m_th_r[k])*m_dt,
         0, 1,  m_v_r[k]*cos(m_th_r[k])*m_dt,
         0, 0,  1;

    B_k<<cos(m_th_r[k])*m_dt, 0,
         sin(m_th_r[k])*m_dt, 0,
         0,                   m_dt;

    C_k<<m_v_r[k]*sin(m_th_r[k])*m_dt*m_th_r[k],
        -m_v_r[k]*cos(m_th_r[k])*m_dt*m_th_r[k],
        0;

    m_A_ltv.push_back(A_k);
    m_B_ltv.push_back(B_k);
    m_C_ltv.push_back(C_k);
  }
}

void linear_mpc::generate_constraint_matrices()
{
  for(int i=0;i<m_N;i++)
  {
    m_A_con.block(3*i,3*i,3,3) = m_A_ltv[i];
    m_A_con.block(3*i,3*i+3,3,3) = -Eigen::MatrixXd::Identity(3,3);
    m_A_con.block(3*i, 3*(m_N+1) + 2*i, 3, 2) = m_B_ltv[i];

    m_b_lb.block(3*i,0,3,1) = -m_C_ltv[i];
    m_b_ub.block(3*i,0,3,1) = -m_C_ltv[i];
  }

  m_A_con.block(3*m_N,0,3,3) = Eigen::MatrixXd::Identity(3,3);
  m_A_con.block(3*m_N+3, 3*m_N+3, 2*m_N, 2*m_N) = Eigen::MatrixXd::Identity(2*m_N,2*m_N);
  m_initial_condition_set = false;

  for (int i=0; i<m_N;i++)
  {
    m_b_lb[3*m_N+3 + 2*i] = -m_v_lim;
    m_b_lb[3*m_N+3 + 2*i+1] = -m_w_lim;

    m_b_ub[3*m_N+3 + 2*i] = m_v_lim;
    m_b_ub[3*m_N+3 + 2*i+1] = m_w_lim;
  }

  //Update F (gradient) vector
  for(int i=0;i<m_N+1;i++)
  {
    m_gradient(3*i)   = -m_Q(0,0) * m_x_r[i];
    m_gradient(3*i+1) = -m_Q(1,1) * m_y_r[i];
    m_gradient(3*i+2) = -m_Q(2,2) * m_th_r[i];
  }
  for(int i=0;i<m_N;i++)
  {
    m_gradient(3*m_N+3 + 2*i)     = -m_R(0,0) * m_v_r[i];
    m_gradient(3*m_N+3 + 2*i + 1) = -m_R(1,1) * m_w_r[i];
  }

  m_gradient(3*m_N+3) += (-m_S(0,0)*m_v_last);
  m_gradient(3*m_N+4) += (-m_S(1,1)*m_w_last);

  m_b_ub(m_dim + 2*m_N - 2) = m_v_last + m_a_lim*m_dt;
  m_b_ub(m_dim + 2*m_N - 1) = m_w_last + m_alpha_lim*m_dt;
  m_b_lb(m_dim + 2*m_N - 2) = m_v_last - m_a_lim*m_dt;
  m_b_lb(m_dim + 2*m_N - 1) = m_w_last - m_alpha_lim*m_dt;
}

void linear_mpc::set_state(double x, double y, double th)
{
  //update the error
  m_initial_condition_set = true;
  Eigen::VectorXd s0(3);
  s0 << x, y, th;

  //set error state initial condition
  m_b_lb.block(3*m_N,0,3,1) = s0;
  m_b_ub.block(3*m_N,0,3,1) = s0;
}

bool linear_mpc::solve(std::vector<UGV::UGVModel::State> &ref)
{
  if (!m_reference_set || !m_initial_condition_set)
  {
    std::cout<<"The optimization problem is not fully constructed."<<std::endl;
    exit(-1);
  }

  if (!m_solver.isInitialized())
  {
    // Generate the sparse matrices
    m_hessian = m_H.sparseView();
    m_linear_constraint = m_A_con.sparseView();

    m_solver.data()->setHessianMatrix(m_hessian);
    m_solver.data()->setGradient(m_gradient);
    m_solver.data()->setLinearConstraintsMatrix(m_linear_constraint);
    m_solver.data()->setLowerBound(m_b_lb);
    m_solver.data()->setUpperBound(m_b_ub);
    m_solver.initSolver();
  }
  else
  {
    // Generate the sparse matrices
    m_linear_constraint = m_A_con.sparseView();
    m_solver.updateLinearConstraintsMatrix(m_linear_constraint);
    m_solver.updateGradient(m_gradient);
    m_solver.updateBounds(m_b_lb,m_b_ub);
  }

  bool success = m_solver.solve();
  if (success)
  {
    m_solution = m_solver.getSolution();

    for (int i=0; i<m_N; i++)
    {
      UGV::UGVModel::State s;
      s.p.x = m_solution((i+1)*3);
      s.p.y = m_solution((i+1)*3 + 1);
      s.theta = m_solution((i+1)*3 + 2);
      s.v = m_solution((m_N+1)*3 + 2*i);
      s.w = m_solution((m_N+1)*3 + 2*i + 1);
      ref.push_back(s);
    }
  }
  return success;

//  v_cmd = m_solution(3*m_N+3);
//  w_cmd = m_solution(3*m_N+4);
}
