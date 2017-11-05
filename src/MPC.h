#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  struct mpc_result_s{
    vector<double> mpc_x; // mpc calculated x points
    vector<double> mpc_y; // mpc calculated x points
    double delta_start;         // delta steering angle
    double a_start;             // throttle
  };
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  mpc_result_s Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
