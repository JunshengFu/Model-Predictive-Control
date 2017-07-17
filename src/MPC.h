#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:

  // max steering angle
  int max_steer;

  /**
   * Constructor
   */
  MPC();

  /**
   * Destructor
   */
  virtual ~MPC();

  /**
   * Solve the model given an initial state and polynomial coefficients.
   * @param state state vector the the vehicle
   * @param coeffs coefficients of the reference line
   * @return Return the first actuator values, and the predicted state of x,y positions
   */
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
