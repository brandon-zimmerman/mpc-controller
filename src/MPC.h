#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {

 double cte_weight_;
 double epsi_weight_;
 double v_weight_;
 double delta_weight_;
 double a_weight_;
 double delta_gap_weight_;
 double a_gap_weight_;
 double ref_velocity_;

 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void init(double cte_weight, double epsi_weight, double v_weight, double delta_weight,
            double a_weight, double delta_gap_weight, double a_gap_weight,
            double ref_velocity);
};

#endif /* MPC_H */
