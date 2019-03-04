#include "PID.h"
#include <vector>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = -1.0; // initialize to first CTE?
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  	// if this is the first cycle, p_error is initialized to -1.0
  	if (p_error < 0.0) p_error = cte;
  
  	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return p_error + d_error + i_error;  
}

double PID::SteerValue() {
	/**
    * Calculate steering angle based on errors
    */
  	return -Kp*p_error - Kd*d_error - Ki*i_error;
}
