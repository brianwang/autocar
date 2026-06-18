#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "autocar_perception/kalman_filter.hpp"

TEST(EKFTest, InitStateIsStored) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 2);
  Eigen::VectorXd x0(3);
  x0 << 0.0, 0.0, 0.0;
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3);
  ekf.setInitialState(x0, P0);
  auto x = ekf.state();
  EXPECT_NEAR(x(0), 0.0, 1e-9);
  EXPECT_NEAR(x(1), 0.0, 1e-9);
}

TEST(EKFTest, PredictionIncreasesUncertainty) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 1);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3) * 0.1;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3) * 0.01;
  ekf.setInitialState(x0, P0);
  ekf.setProcessNoise(Q);

  auto before = ekf.covariance().trace();
  Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
  ekf.predict(u, 0.1,
    [](const auto& s, const auto&, double) { return s; },
    [](const auto& s, const auto&, double) { return Eigen::MatrixXd::Identity(3, 3); });
  auto after = ekf.covariance().trace();
  EXPECT_GT(after, before);
}

TEST(EKFTest, UpdateReducesUncertainty) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 1);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3) * 1.0;
  ekf.setInitialState(x0, P0);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  ekf.setMeasurementNoise(R);

  auto before = ekf.covariance().trace();
  Eigen::VectorXd z(1); z << 0.5;
  ekf.update(z,
    [](const auto& s) { return Eigen::VectorXd::Constant(1, s(0)); },
    [](const auto& s) {
      Eigen::MatrixXd H(1, 3);
      H << 1.0, 0.0, 0.0;
      return H;
    });
  auto after = ekf.covariance().trace();
  EXPECT_LT(after, before);
}
