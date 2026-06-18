#include "autocar_perception/kalman_filter.hpp"
#include <stdexcept>

namespace autocar_perception {

ExtendedKalmanFilter::ExtendedKalmanFilter(int state_dim, int measurement_dim)
  : state_dim_(state_dim), measurement_dim_(measurement_dim) {
  x_ = Eigen::VectorXd::Zero(state_dim);
  P_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
  R_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.1;
}

void ExtendedKalmanFilter::setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
  if (x0.size() != state_dim_) throw std::invalid_argument("state dim mismatch");
  x_ = x0;
  P_ = P0;
}

void ExtendedKalmanFilter::setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
void ExtendedKalmanFilter::setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

void ExtendedKalmanFilter::predict(
    const Eigen::VectorXd& u, double dt,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> f,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> F_jacobian) {
  x_ = f(x_, u, dt);
  Eigen::MatrixXd F = F_jacobian(x_, u, dt);
  P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H_jacobian) {
  Eigen::VectorXd z_pred = h(x_);
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd H = H_jacobian(x_);
  Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  P_ = (I - K * H) * P_;
}

}  // namespace autocar_perception
