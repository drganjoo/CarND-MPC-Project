#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve_result.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  bool Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  inline double GetSteering() {
    const double steering_to_1 = deg2rad(25.0) * Lf;
    return -solution.x[delta_start] / steering_to_1;
  }

  inline double GetThrottle() {
    return solution.x[a_start];
  }

  void GetPredictedPoints(vector<double> &x, vector<double> &y);
  void GetPolyFitPoints(Eigen::VectorXd &coeffs, vector<double> &x, vector<double> &y);

  inline double deg2rad(double x) { return x * M_PI / 180; }
  inline double rad2deg(double x) { return x * 180 / M_PI; }

  typedef CPPAD_TESTVECTOR(double) Dvector;
  CppAD::ipopt::solve_result<Dvector> solution;
};

#endif /* MPC_H */
