#ifndef ltv_mpc_filter_H
#define ltv_mpc_filter_H
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <cpc_motion_planning/ugv/model/ugv_model.h>

class ltv_mpc_filter
{
public:
  ltv_mpc_filter(double dt, int N, double v_lim, double w_lim, double a_lim, double alpha_lim);
  void set_reference_and_state(const std::vector<double> &x_r, const std::vector<double> &y_r,
                     const std::vector<double> &th_r, const std::vector<double> &v_r,
                     const std::vector<double> &w_r, double x, double y, double th, double v_last, double w_last);
  void set_cost(const Eigen::Matrix<double,3,3> &Q, const Eigen::Matrix<double,2,2> &R, const Eigen::Matrix<double, 2, 2> &S);
  void set_state(double x, double y, double th);
  bool solve(std::vector<UGV::UGVModel::State> &ref);

private:
  void generate_ltv_model();
  void generate_constraint_matrices();
  double in_pi(double in) const
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  void un_in_pi_ref(double th)
  {
    m_th_r[0] = in_pi(m_th_r[0] - th) + th;

    for(size_t i=1; i<m_th_r.size(); i++)
    {
      m_th_r[i] = in_pi(m_th_r[i] - m_th_r[i-1]) + m_th_r[i-1];
    }
  }
private:
  int m_N;
  int m_dim;
  double m_dt;
  Eigen::MatrixXd m_H;
  Eigen::SparseMatrix<double> m_hessian, m_linear_constraint;
  std::vector<Eigen::Matrix<double,3,3>> m_A_ltv;
  std::vector<Eigen::Matrix<double,3,2>> m_B_ltv;
  std::vector<Eigen::Matrix<double,3,1>> m_C_ltv;
  Eigen::Matrix<double,3,3> m_Q;
  Eigen::Matrix<double,2,2> m_R;
  Eigen::Matrix<double,2,2> m_S;
  std::vector<double> m_x_r, m_y_r, m_th_r, m_v_r, m_w_r;
  Eigen::MatrixXd m_A_con;
  Eigen::VectorXd m_b_lb, m_b_ub;
  bool m_initial_condition_set, m_reference_set;
  OsqpEigen::Solver m_solver;
  Eigen::VectorXd m_gradient;
  Eigen::VectorXd m_solution;
  double m_v_lim, m_w_lim;
  double m_v_last, m_w_last;
  double m_a_lim, m_alpha_lim;
};

#endif // ltv_mpc_filter_H
