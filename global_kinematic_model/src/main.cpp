// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"

//
// Helper functions
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());

  // Assign to local variables for readability
  double x_p = state[0];
  double y_p = state[1];
  double psi_p = state[2];
  double v_p = state[3];
  double delta = actuators[0];
  double a = actuators[1];

  double x, y, psi, v;

  x = x_p + v_p*cos(psi_p)*dt;
  y = y_p + v_p*sin(psi_p)*dt;
  psi = psi_p + v_p/Lf*delta*dt;
  v = v_p + a*dt;

  // map to output
  next_state[0] = x;
  next_state[1] = y;
  next_state[2] = psi;
  next_state[3] = v;

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  // should be [0.212132, 0.212132, 0.798488, 1.3]
  auto next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}
