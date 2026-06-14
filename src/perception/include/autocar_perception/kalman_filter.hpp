#pragma once

#include <Eigen/Dense>

namespace autocar_perception {

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter(int state_dim, int measurement_dim);

  void setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
  void setProcessNoise(const Eigen::MatrixXd& Q);
  void setMeasurementNoise(const Eigen::MatrixXd& R);

  void predict(const Eigen::VectorXd& u, double dt,
               std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> f,
               std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> F_jacobian);

  void update(const Eigen::VectorXd& z,
              std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
              std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H_jacobian);

  Eigen::VectorXd state() const { return x_; }
  Eigen::MatrixXd covariance() const { return P_; }

private:
  int state_dim_;
  int measurement_dim_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

}  // namespace autocar_perception
