#include "autocar_control/pid_controller.hpp"
#include <algorithm>

namespace autocar_control {

PIDController::PIDController(double kp, double ki, double kd,
                             double output_min, double output_max,
                             double integral_limit)
  : kp_(kp), ki_(ki), kd_(kd),
    output_min_(output_min), output_max_(output_max),
    integral_limit_(integral_limit) {}

void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_run_ = true;
}

double PIDController::compute(double setpoint, double measurement, double dt) {
  if (dt <= 0.0) return 0.0;

  double error = setpoint - measurement;

  if (first_run_) {
    prev_error_ = error;
    first_run_ = false;
  }

  double p_term = kp_ * error;

  integral_ += error * dt;
  integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
  double i_term = ki_ * integral_;

  double derivative = (error - prev_error_) / dt;
  double d_term = kd_ * derivative;
  prev_error_ = error;

  double output = p_term + i_term + d_term;
  output = std::clamp(output, output_min_, output_max_);

  return output;
}

void PIDController::setGains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

}  // namespace autocar_control
