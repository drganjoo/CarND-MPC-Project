#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

//#define _USE_LATENCY_CONSTRAINT

using CppAD::AD;

size_t N = 25;
double dt = 0.1;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
const double ref_v = 40;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    const double kCte = 4000;  // 2000
    const double kEpsi = 2000; // 2000
    const double kV = 1; // 1
    const double kSteering = 5; // 5
    const double kAcceleration = 5;  // 5
    const double kSteeringChange = 200; // 200
    const double kAccelChange = 10; // 10

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += kCte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += kEpsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += kV * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += kSteering * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += kAcceleration * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += kSteeringChange * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += kAccelChange * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    #ifdef _USE_LATENCY_CONSTRAINT
    fg[1 + delta_start] = vars[delta_start];
    fg[1 + a_start] = vars[a_start];
    #endif

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // psi_desired = f'(x), 3rd order polynomial = a + bx + cx^2 + dx^3
      // f'(x) of 3rd of polynomial = b + 2 * cx + 3 * dx^2
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      auto f_derivative = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2);
      AD<double> psides0 = CppAD::atan(f_derivative);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

      // put in a constraint that the acceleration and steering for every alternate
      // time T won't be changed
#ifdef _USE_LATENCY_CONSTRAINT
      // for every other 100ms we need to make sure that there is a constraint
      // to use the same value as before

      if (t % 2 == 1) {
        AD<double> delta1 = vars[delta_start + t];
        AD<double> a1 = vars[a_start + t];

        cout << "Constraint @: " << 1 + delta_start + t - 1 << " for time: " << t << endl;
        fg[1 + delta_start + t - 1] = delta1 - delta0;
        fg[1 + delta_start + t] = a1 - a0;
      }
#endif
    }

    cout << "Going out" << endl;
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  cout << "Using N: " << N << " DT: " << dt << " Ref speed: " << ref_v << endl;
#ifdef _USE_LATENCY_CONSTRAINT
  cout << "Using latency constraints" << endl;
#endif
}

MPC::~MPC() {

}

Result MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  Result res;
  res.ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  size_t n_vars = N * 6 + (N - 1) * 2;

#ifdef _USE_LATENCY_CONSTRAINT
  double delta = state[6];
  double a = state[7];

  // The value set for delta and acceleration will remain the same
  // for the next 100ms, then it can change and then again for the next
  // 100ms the value cannot change. Therefore there will be
  // ((N - 1) / 2) * 2 more constraints.

  size_t n_constraints = N * 6 + (N - 1);
#else
  size_t n_constraints = N * 6;
#endif

  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  #ifdef _USE_LATENCY_CONSTRAINT
  vars[delta_start] = delta;
  vars[a_start] = a;
  #endif

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 in radians
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 0.6;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

#ifdef _USE_LATENCY_CONSTRAINT
  constraints_lowerbound[delta_start] = delta;
  constraints_upperbound[delta_start] = delta;
  constraints_lowerbound[a_start] = a;
  constraints_upperbound[a_start] = a;
#endif

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  res.ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  res.cost = solution.obj_value;
  res.steering = solution.x[delta_start];
  res.throttle = solution.x[a_start];

  for (size_t i = 1; i < N; i++) {
    res.x.push_back(solution.x[x_start + i]);
    res.y.push_back(solution.x[y_start + i]);
  }

  return res;
}
