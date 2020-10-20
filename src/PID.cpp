#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_error = 0.0;
  prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return - Kp*p_error - Kd*d_error - Ki*i_error;  // TODO: Add your total error calc here!
}

void PID::resetErrors(){
  i_error = 0.0;
  prev_cte = 0.0;
}

void PID::setParams(vector<double> new_params){
  Kp = new_params[0];
  Kd = new_params[1];
  Ki = new_params[2];
}