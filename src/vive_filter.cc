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

bool ViveFilter::Predict(const sensor_msgs::Imu & msg) {
  // Convert measurements
  Eigen::Vector3d lin_acc = filter::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d ang_vel = filter::ConvertMessage(msg.angular_velocity);

  // Update matrix
  Eigen::Matrix<double, 13, 13> F;
  F.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  F.block<3,7>(0,6) = Eigen::MatrixXd::Zero(3,7);
  F.block<10,6>(3,0) = Eigen::MatrixXd::Zero(10,6);
  F.block<3,3>(10,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,7>(10,6) = Eigen::MatrixXd::Zero(3,7);

  // Eigen::Matrix<double, 3, 4> Fvq;
  // double Fvq0 = 2*(orientation_.w() * a_m(0) -
  //   orientation_.z() * a_m(1) +
  //   orientation_.y() * a_m(2))
  // double Fvq1 = 2*(orientation_.x() * a_m(0) +
  //   orientation_.y() * a_m(1) +
  //   orientation_.z() * a_m(2))
  // double Fvq2 = 2*( - orientation_.y() * a_m(0) +
  //   orientation_.x() * a_m(1) +
  //   orientation_.w() * a_m(2))
  // double Fvq3 = 2*( - orientation_.z() * a_m(0) -
  //   orientation_.w() * a_m(1) +
  //   orientation_.x() * a_m(2))
  // Fvq(0,0) = Fvq0;
  // Fvq(0,1) = Fvq1;
  // Fvq(0,2) = Fvq2;
  // Fvq(0,3) = Fvq3;
  // Fvq(1,0) = - Fvq3;
  // Fvq(1,1) = - Fvq2;
  // Fvq(1,2) = Fvq1;
  // Fvq(1,3) = Fvq0;
  // Fvq(2,0) = Fvq2;
  // Fvq(2,1) = - Fvq3;
  // Fvq(2,2) = - Fvq0;
  // Fvq(2,3) = Fvq1;
  // F.block<3,4>(3,6) = Fvq;

  // Eigen::Matrix<double, 4, 4> Fqq;
  // Fqq(0,0) = 0.0;
  // Fqq(0,1) = w_bias_(0) - a_m(0);
  // Fqq(0,2) = w_bias_(1) - a_m(1);
  // Fqq(0,3) = w_bias_(2) - a_m(2);
  // Fqq(1,0) = ?;
  // Fqq(1,1) = 0.0;
  // Fqq(1,2) = ?;
  // Fqq(1,3) = w_bias_(1) - a_m(1);
  // Fqq(2,0) = ?;
  // Fqq(2,1) = w_bias_(2) - a_m(2);
  // Fqq(2,2) = 0.0;
  // Fqq(2,3) = ?;
  // Fqq(3,0) = ?;
  // Fqq(3,1) = ?;
  // Fqq(3,2) = w_bias_(0) - a_m(0);
  // Fqq(3,3) = 0.0;

  // Eigen::Matrix<double, 4, 3> Fqb;
  // Fqb(0,0) = - orientation_.x();
  // Fqb(0,1) = - orientation_.y();
  // Fqb(0,2) = - orientation_.z();
  // Fqb(1,0) = orientation_.w();
  // Fqb(1,1) = - orientation_.z();
  // Fqb(1,2) = orientation_.y();
  // Fqb(2,0) = orientation_.z();
  // Fqb(2,1) = orientation_.w();
  // Fqb(2,2) = - orientation_.x();
  // Fqb(3,0) = - orientation_.y();
  // Fqb(3,1) = orientation_.x();
  // Fqb(3,2) = orientation_.w();

  return true;
}

bool ViveFilter::Update(const hive::ViveLight & msg) {
  return true;
}

int main(int argc, char ** argv)
{
  ROS_INFO("FILTERING");
  return 0;
}