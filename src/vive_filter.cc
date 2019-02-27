// ROS includes
#include <hive/vive_filter.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <thread>

ViveFilter::ViveFilter() {
  position_ << 0.0, 0.0, 0.0;
  velocity_ << 0.0, 0.0, 0.0;
  orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  w_bias_ << 0.0, 0.0, 0.0;
  gravity_ << 0.0, 0.0, 9.8;
  // transform from imu frame to light frame
  imu_transform_.first << 0.0, 0.0, 0.0;
  imu_transform_.second << Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  return;
}

ViveFilter::~ViveFilter() {
  return;
}

void ViveSolve::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  // Measured linear accelaration
  Eigen::Vector3 a_m << msg->linear_acceleration.x,
  msg->linear_acceleration.y,
  msg->linear_acceleration.z;
  // Measured angular accelaration
  Eigen::Vector3 w_m << msg->angular_velocity.x,
  msg->angular_velocity.y,
  msg->angular_velocity.z;
  // Update matrix
  Eigen::Matrix<double, 13, 13> F;
  F.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  F.block<3,7>(0,6) = Eigen::MatrixXd::Zero(3,7);
  F.block<10,6>(3,0) = Eigen::MatrixXd::Zero(10,6);
  F.block<3,3>(10,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,7>(10,6) = Eigen::MatrixXd::Zero(3,7);

  Eigen::Matrix<double, 3, 4> Fvq;
  double Fvq0 = 2*(orientation_.w() * a_m(0) -
    orientation_.z() * a_m(1) +
    orientation_.y() * a_m(2))
  double Fvq1 = 2*(orientation_.x() * a_m(0) +
    orientation_.y() * a_m(1) +
    orientation_.z() * a_m(2))
  double Fvq2 = 2*( - orientation_.y() * a_m(0) +
    orientation_.x() * a_m(1) +
    orientation_.w() * a_m(2))
  double Fvq3 = 2*( - orientation_.z() * a_m(0) -
    orientation_.w() * a_m(1) +
    orientation_.x() * a_m(2))
  Fvq(0,0) = Fvq0;
  Fvq(0,1) = Fvq1;
  Fvq(0,2) = Fvq2;
  Fvq(0,3) = Fvq3;
  Fvq(1,0) = - Fvq3;
  Fvq(1,1) = - Fvq2;
  Fvq(1,2) = Fvq1;
  Fvq(1,3) = Fvq0;
  Fvq(2,0) = Fvq2;
  Fvq(2,1) = - Fvq3;
  Fvq(2,2) = - Fvq0;
  Fvq(2,3) = Fvq1;
  F.block<3,4>(3,6) = Fvq;

  Eigen::Matrix<double, 4, 4> Fqq;
  Fqq(0,0) = 0.0;
  Fqq(0,1) = w_bias_(0) - a_m(0);
  Fqq(0,2) = w_bias_(1) - a_m(1);
  Fqq(0,3) = w_bias_(2) - a_m(2);
  Fqq(1,0) = ?;
  Fqq(1,1) = 0.0;
  Fqq(1,2) = ?;
  Fqq(1,3) = w_bias_(1) - a_m(1);
  Fqq(2,0) = ?;
  Fqq(2,1) = w_bias_(2) - a_m(2);
  Fqq(2,2) = 0.0;
  Fqq(2,3) = ?;
  Fqq(3,0) = ?;
  Fqq(3,1) = ?;
  Fqq(3,2) = w_bias_(0) - a_m(0);
  Fqq(3,3) = 0.0;

  Eigen::Matrix<double, 4, 3> Fqb;
  Fqb(0,0) = - orientation_.x();
  Fqb(0,1) = - orientation_.y();
  Fqb(0,2) = - orientation_.z();
  Fqb(1,0) = orientation_.w();
  Fqb(1,1) = - orientation_.z();
  Fqb(1,2) = orientation_.y();
  Fqb(2,0) = orientation_.z();
  Fqb(2,1) = orientation_.w();
  Fqb(2,2) = - orientation_.x();
  Fqb(3,0) = - orientation_.y();
  Fqb(3,1) = orientation_.x();
  Fqb(3,2) = orientation_.w();

  return;
}

void ViveSolve::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  return;
}