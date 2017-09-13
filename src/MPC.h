#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve_result.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Result{
  bool ok;
  double cost;
  double steering;
  double throttle;
  vector<double> x;
  vector<double> y;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
