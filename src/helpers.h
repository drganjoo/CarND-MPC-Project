//
// Created by Fahad Zubair on 9/13/17.
//

#ifndef MPC_HELPERS_H
#define MPC_HELPERS_H

#include <string>
#include "Eigen-3.3/Eigen/Core"

const size_t N = 15;
const double dt = 0.1;
const double Lf = 2.67;
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

std::string hasData(std::string s);
double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif //MPC_HELPERS_H
