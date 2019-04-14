// ROS includes
#include <hive/vive_filter.h>

namespace filter {
  Eigen::Vector3d ConvertMessage(geometry_msgs::Vector3 msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
  }

  Eigen::Quaterniond ConvertMessage(geometry_msgs::Quaternion msg) {
    return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
  }

  Eigen::Matrix<double, 13, 13> ConvertMessage(double * cov) {
    Eigen::Matrix<double, 13, 13> M;
    for (size_t i = 0; i < 13; i++) {
      for (size_t j = 0; j < 13; j++) {
        M(i,j) = cov[i*13 + j];
      }
    }
    return M;
  }
}

ViveFilter::ViveFilter() {
  // Position initialization
  position_.x = 0.0;
  position_.y = 0.0;
  position_.z = 0.0;
  // Velocity initialization
  velocity_.x = 0.0;
  velocity_.y = 0.0;
  velocity_.z = 0.0;
  // Quaternion initialization
  rotation_.w = 1.0;
  rotation_.x = 0.0;
  rotation_.y = 0.0;
  rotation_.z = 0.0;
  // Bias initialization
  bias_.x = 0.0;
  bias_.y = 0.0;
  bias_.z = 0.0;
  // Gravity
  gravity_.x = 0.0;
  gravity_.y = 0.0;
  gravity_.z = -9.8;
  return;
}

ViveFilter::~ViveFilter() {
  return;
}

void ViveFilter::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  Predict(*msg);
  return;
}

void ViveFilter::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  return;
}

bool ViveFilter::GetTransform(geometry_msgs::TransformStamped& msg) {
  return true;
}

// Derivative of the Vive's inertial model
Eigen::MatrixXd GetF(Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity,
  Eigen::Vector3d bias) {

  // Set matrix
  Eigen::Matrix<double, 10, 10> F;
  F.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  F.block<3,4>(0,6) = Eigen::MatrixXd::Zero(3,4);
  F.block<3,3>(3,0) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(3,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<4,3>(6,0) = Eigen::MatrixXd::Zero(4,3);
  F.block<4,3>(6,3) = Eigen::MatrixXd::Zero(4,3);

  // d\dot{V}/dq
  F(3,6) = 2*linear_acceleration(2) * rotation.y() -
    2*linear_acceleration(1) * rotation.z();
  F(4,6) = 2*linear_acceleration(0) * rotation.z() -
    2*linear_acceleration(2) * rotation.x();
  F(5,6) = 2*linear_acceleration(1) * rotation.x() -
    2*linear_acceleration(0) * rotation.y();

  F(3,7) = 2*linear_acceleration(1) * rotation.y() +
    2*linear_acceleration(2) * rotation.z();
  F(4,7) = 2*linear_acceleration(0) * rotation.y() -
    4*linear_acceleration(1) * rotation.x() -
    2*linear_acceleration(2) * rotation.w();
  F(5,7) = 2*linear_acceleration(1) * rotation.w() +
    2*linear_acceleration(0) * rotation.z() -
    4*linear_acceleration(2) * rotation.x();

  F(3,8) = 2*linear_acceleration(1) * rotation.x() -
    4*linear_acceleration(0) * rotation.y() +
    2*linear_acceleration(2) * rotation.w();
  F(4,8) = 2*linear_acceleration(0) * rotation.x() +
    2*linear_acceleration(2) * rotation.z();
  F(5,8) = 2*linear_acceleration(1) * rotation.z() -
    2*linear_acceleration(0) * rotation.w() -
    4*linear_acceleration(2) * rotation.y();

  F(3,9) = 2*linear_acceleration(2) * rotation.x() -
    4*linear_acceleration(0) * rotation.z() -
    2*linear_acceleration(1) * rotation.w();
  F(4,9) = 2*linear_acceleration(0) * rotation.w() -
    4*linear_acceleration(1) * rotation.z() +
    2*linear_acceleration(2) * rotation.y();
  F(5,9) = 2*linear_acceleration(0) * rotation.x() +
    2*linear_acceleration(1) * rotation.y();

  // d\dot{q}/dq
  F(6,6) = 0.0;
  F(7,6) = angular_velocity(0)/2;
  F(8,6) = angular_velocity(1)/2;
  F(9,6) = angular_velocity(2)/2;

  F(6,7) = -angular_velocity(0)/2;
  F(7,7) = 0.0;
  F(8,7) = -angular_velocity(2)/2;
  F(9,7) = angular_velocity(1)/2;

  F(6,8) = -angular_velocity(1)/2;
  F(7,8) = angular_velocity(2)/2;
  F(8,8) = 0.0;
  F(9,8) = -angular_velocity(0)/2;

  F(6,9) = -angular_velocity(2)/2;
  F(7,9) = -angular_velocity(1)/2;
  F(8,9) = angular_velocity(0)/2;
  F(9,9) = 0.0;

  return F;
}

bool ViveFilter::Predict(const sensor_msgs::Imu & msg) {
  // Convert measurements
  Eigen::Vector3d meas_lin_acc = filter::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d meas_ang_vel = filter::ConvertMessage(msg.angular_velocity);
  Eigen::Quaterniond rotation = filter::ConvertMessage(rotation_);
  Eigen::Vector3d bias = filter::ConvertMessage(bias_);

  // Update matrix
  Eigen::MatrixXd F = GetF(rotation,
    meas_lin_acc,
    meas_ang_vel,
    bias);

  std::cout << F << std::endl;

  return true;
}

bool ViveFilter::Update(const hive::ViveLight & msg) {
  return true;
}

int main(int argc, char ** argv)
{
  ROS_INFO("FILTERING");

  sensor_msgs::Imu msg;
  ViveFilter filter;
  filter.Predict(msg);

  return 0;
}