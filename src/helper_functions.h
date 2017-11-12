/*
 * helper_functions.h
 *
 *  Created on: Nov 4, 2017
 *      Author: artur
 */
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/QR"

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

using Eigen::VectorXd;

inline void veh2map(const vector<double>& ptsx,
             const vector<double>& ptsy,
             const double px,
             const double py,
             const double psi,
             VectorXd& x,
             VectorXd& y){

  x.resize(ptsx.size());
  y.resize(ptsy.size());

  for (size_t i = 0; i < ptsx.size(); i++) {
    //shift car reference angle to 90 degrees
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;
    x[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    y[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
  }
}

//calculate derivative
inline double der2(double x, const Eigen::VectorXd& coeffs){
    double d = 0;
    for (int i = 1; i < coeffs.size(); i++)
        d += pow(x, i - 1) * i * coeffs[i];
    return d;
}
// Evaluate a polynomial.
inline double polyeval(const Eigen::VectorXd& coeffs, const double& x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals,
                               const Eigen::VectorXd& yvals,
                               const int& order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


#endif /* HELPER_FUNCTIONS_H_ */
