#pragma once

namespace autocar_control {

class PIDController {
public:
  PIDController(double kp, double ki, double kd,
                double output_min, double output_max,
                double integral_limit);

  void reset();
  double compute(double setpoint, double measurement, double dt);
  void setGains(double kp, double ki, double kd);

private:
  double kp_, ki_, kd_;
  double output_min_, output_max_;
  double integral_limit_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool first_run_{true};
};

}  // namespace autocar_control
