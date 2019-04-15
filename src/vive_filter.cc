// ROS includes
#include <hive/vive_filter.h>

#define STATE_SIZE 10

namespace filter {
  Eigen::Vector3d ConvertMessage(geometry_msgs::Vector3 msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
  }

  Eigen::Quaterniond ConvertMessage(geometry_msgs::Quaternion msg) {
    return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
  }

  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> ConvertMessage(double * cov) {
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> M;
    for (size_t i = 0; i < STATE_SIZE; i++) {
      for (size_t j = 0; j < STATE_SIZE; j++) {
        M(i,j) = cov[i*STATE_SIZE + j];
      }
    }
    return M;
  }

  geometry_msgs::Vector3 ConvertMessage(Eigen::Vector3d v) {
    geometry_msgs::Vector3 msg;
    msg.x = v(0);
    msg.y = v(1);
    msg.z = v(2);
    return msg;
  }

  geometry_msgs::Quaternion ConvertMessage(Eigen::Quaterniond q) {
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }
}

ViveFilter::ViveFilter() {
  // Position initialization
  position_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  // Velocity initialization
  velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Quaternion initialization
  rotation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  // Bias initialization
  bias_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Gravity
  gravity_ = Eigen::Vector3d(0.0, 0.0, -9.8);
  // Model covariance
  model_covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE);
  // Measure covariance - assuming 3 sensors for now
  measure_covariance_ = Eigen::MatrixXd::Identity(3,3);
  // Aux variables
   covariance_ = model_covariance_;
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
  msg.transform.translation.x = position_(0);
  msg.transform.translation.y = position_(1);
  msg.transform.translation.z = position_(2);
  msg.transform.rotation.w = rotation_.w();
  msg.transform.rotation.x = rotation_.x();
  msg.transform.rotation.y = rotation_.y();
  msg.transform.rotation.z = rotation_.z();
  msg.child_frame_id = tracker_.serial;
  msg.header.stamp = time_;
  msg.header.frame_id = "vive"; // For now it's not this the frame
  return true;
}

// Derivative of the Vive's inertial model
Eigen::MatrixXd GetF(Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity,
  Eigen::Vector3d bias) {

  // Set matrix
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;
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

// Derivative of the measurement model - Horizontal measures
Eigen::MatrixXd GetHorizontalH(Eigen::Vector3d translation,
  Eigen::Quaterniond rotation,
  std::vector<Eigen::Vector3d> photodiodes) {
  Eigen::MatrixXd H = Eigen::MatrixXd(photodiodes.size(), STATE_SIZE);
  // Convert to shorter notation
  double Px = translation(0),
    Py = translation(1),
    Pz = translation(2);
  double qw = rotation.w(),
    qx = rotation.x(),
    qy = rotation.y(),
    qz = rotation.z();
  size_t row = 0;
  // Iterate over all photodiodes
  for (auto photodiode : photodiodes) {
    double ppx = photodiode(0),
      ppy = photodiode(1),
      ppz = photodiode(2);
    // d alpha / d Px
    H(row, 0) = 1.0/((pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0)*(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0)));
    // d alpha / d Py
    H(row, 1) = 0.0;
    // d alpha / d Pz
    H(row, 2) = -((Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d V
    H(row, 3) = 0.0;
    H(row, 4) = 0.0;
    H(row, 5) = 0.0;
    // d alpha / d Qw
    H(row, 6) = ((ppz*qy*2.0-ppy*qz*2.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))-(ppy*qx*2.0-ppx*qy*2.0)*(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qx
    H(row, 7) = ((ppy*qy*2.0+ppz*qz*2.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))-(ppy*qw*2.0-ppz*qx*4.0+ppx*qz*2.0)*(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qy
    H(row, 8) = ((ppz*qw*2.0+ppy*qx*2.0-ppx*qy*4.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))+(ppx*qw*2.0+ppz*qy*4.0-ppy*qz*2.0)*(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qz
    H(row, 9) = -((ppy*qw*2.0-ppz*qx*2.0+ppx*qz*4.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))+(ppx*qx*2.0+ppy*qy*2.0)*(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Px-ppx*((qy*qy)*2.0+(qz*qz)*2.0-1.0)-ppy*(qw*qz*2.0-qx*qy*2.0)+ppz*(qw*qy*2.0+qx*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // row
    row++;
  }
  return H;
}

// Derivative of the measurement model - Vertical measures
Eigen::MatrixXd GetVerticalH(Eigen::Vector3d translation,
  Eigen::Quaterniond rotation,
  std::vector<Eigen::Vector3d> photodiodes) {
  Eigen::MatrixXd H = Eigen::MatrixXd(photodiodes.size(), STATE_SIZE);
  double Px = translation(0),
    Py = translation(1),
    Pz = translation(2);
  double qw = rotation.w(),
    qx = rotation.x(),
    qy = rotation.y(),
    qz = rotation.z();
  size_t row = 0;
  for (auto photodiode : photodiodes) {
    double ppx = photodiode(0),
      ppy = photodiode(1),
      ppz = photodiode(2);
    // d alpha / d Px
    H(row, 0) = 0.0;
    // d alpha / d Py
    H(row, 1) = 1.0/((pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0)*(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0)));
    // d alpha / d Pz
    H(row, 2) = -((Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d V
    H(row, 3) = 0.0;
    H(row, 4) = 0.0;
    H(row, 5) = 0.0;
    // d alpha / d Qw
    H(row, 6) = -((ppz*qx*2.0-ppx*qz*2.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))+(ppy*qx*2.0-ppx*qy*2.0)*(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qx
    H(row, 7) = -((ppz*qw*2.0+ppy*qx*4.0-ppx*qy*2.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))+(ppy*qw*2.0-ppz*qx*4.0+ppx*qz*2.0)*(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qy
    H(row, 8) = ((ppx*qx*2.0+ppz*qz*2.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))+(ppx*qw*2.0+ppz*qy*4.0-ppy*qz*2.0)*(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // d alpha / d Qz
    H(row, 9) = ((ppx*qw*2.0+ppz*qy*2.0-ppy*qz*4.0)/(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0))-(ppx*qx*2.0+ppy*qy*2.0)*(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0))*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0))/(pow(Py-ppy*((qx*qx)*2.0+(qz*qz)*2.0-1.0)+ppx*(qw*qz*2.0+qx*qy*2.0)-ppz*(qw*qx*2.0-qy*qz*2.0),2.0)*1.0/pow(Pz-ppz*((qx*qx)*2.0+(qy*qy)*2.0-1.0)-ppx*(qw*qy*2.0-qx*qz*2.0)+ppy*(qw*qx*2.0+qy*qz*2.0),2.0)+1.0);
    // row
    row++;
  }
  return H;
}

// Auxility matrix
Eigen::MatrixXd GetOmega(Eigen::Quaterniond Q) {
  Eigen::Matrix<double, 4, 3> Omega;
  Omega(0,0) = -Q.x();
  Omega(0,1) = -Q.y();
  Omega(0,2) = -Q.z();
  Omega(1,0) = Q.w();
  Omega(1,1) = -Q.z();
  Omega(1,2) = Q.y();
  Omega(2,0) = Q.z();
  Omega(2,1) = Q.w();
  Omega(2,2) = -Q.x();
  Omega(3,0) = -Q.y();
  Omega(3,1) = Q.x();
  Omega(3,2) = Q.w();
  return Omega;
}

// Vector that represents the evolution of the system is continuous time
Eigen::MatrixXd GetDState(Eigen::Vector3d velocity,
  Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity) {
  Eigen::Matrix<double, STATE_SIZE, 1> dotX;
  // Position
  dotX.block<3,1>(0,0) = velocity;
  // Velocity - assumed constant
  dotX.block<3,1>(3,0) = rotation.toRotationMatrix() * linear_acceleration;
  // Orientation
  Eigen::MatrixXd Omega = GetOmega(rotation);
  dotX.block<4,1>(6,0) = 0.5 * (Omega * angular_velocity);
  return dotX;
}

// Vector that represents the horizontal predicted measurements according to the
// system's state
Eigen::MatrixXd GetHorizontalZ(Eigen::Vector3d position,
  Eigen::Quaterniond rotation,
  std::vector<Eigen::Vector3d> photodiodes) {
  // Declaring measurements vector
  Eigen::MatrixXd Z = Eigen::MatrixXd(photodiodes.size(),1);
  size_t row = 0;
  for (auto photodiode : photodiodes) {
    Eigen::MatrixXd lPp = rotation.toRotationMatrix() * photodiode + position;
    Z(row) = atan2(lPp(0),lPp(2));
    row++;
  }
  return Z;
}

// Vector that represents the vertical predicted measurements according to the
// system's state
Eigen::MatrixXd GetVerticalZ(Eigen::Vector3d position,
  Eigen::Quaterniond rotation,
  std::vector<Eigen::Vector3d> photodiodes) {
  // Declaring measurements vector
  Eigen::MatrixXd Z = Eigen::MatrixXd(photodiodes.size(),1);
  size_t row = 0;
  for (auto photodiode : photodiodes) {
    Eigen::MatrixXd lPp = rotation.toRotationMatrix() * photodiode + position;
    Z(row) = atan2(lPp(1),lPp(2));
    row++;
  }
  return Z;
}

// Time update (Inertial data)
bool ViveFilter::Predict(const sensor_msgs::Imu & msg) {
  // Convert measurements
  Eigen::Vector3d linear_acceleration = filter::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d angular_velocity = filter::ConvertMessage(msg.angular_velocity);

  // Time difference
  double dT = (msg.header.stamp - time_).toSec();

  // Derivative of the state in time
  Eigen::MatrixXd dState = GetDState(velocity_,
    rotation_,
    linear_acceleration,
    angular_velocity);
  // Old state
  Eigen::Matrix<double, STATE_SIZE, 1> oldX;
  oldX.block<3,1>(0,0) = position_;
  oldX.block<3,1>(3,0) = velocity_;
  oldX.block<4,1>(6,0) = Eigen::Vector4d(
    rotation_.w(),
    rotation_.x(),
    rotation_.y(),
    rotation_.z());
  // New state
  Eigen::MatrixXd newX = oldX + dT * dState;
  // Conversion of the new state
  position_ = newX.block<3,1>(0,0);
  velocity_ = newX.block<3,1>(3,0);
  rotation_ = Eigen::Quaterniond(newX(6), newX(7), newX(8),
    newX(9)).normalized();

  // Covariance update
  Eigen::MatrixXd oldP = covariance_;
  Eigen::MatrixXd F = GetF(rotation_,
    linear_acceleration,
    angular_velocity,
    bias_);
  Eigen::MatrixXd Q = model_covariance_;
  Eigen::MatrixXd newP = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) + dT * F) *
  oldP * (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) + dT * F).transpose() + Q;
  return true;
}

Eigen::MatrixXd SliceR(Eigen::MatrixXd R,
  std::vector<int> sensors) {
  size_t col = 0, row = 0;
  Eigen::MatrixXd slicedR = Eigen::MatrixXd(sensors.size(),
    sensors.size());
  for (auto sensor_row : sensors) {
    for (auto sensor_col : sensors) {
      slicedR(row, col) = R(sensor_row, sensor_col);
      col++;
    }
    row++;
  }
  return slicedR;
}

// Measure upate (Light data)
bool ViveFilter::Update(const hive::ViveLight & msg) {
  size_t row = 0;
  std::vector<Eigen::Vector3d> photodiodes;
  Eigen::MatrixXd Z = Eigen::MatrixXd(msg.samples.size(),1);
  std::vector<int> sensors;
  for (auto sample : msg.samples) {
    // Add photodiode's position to vector
    photodiodes.push_back(Eigen::Vector3d(
      tracker_.sensors[sample.sensor].position.x,
      tracker_.sensors[sample.sensor].position.y,
      tracker_.sensors[sample.sensor].position.z));
    // Put angle in Vector
    Z(row, 0) = sample.angle;
    row++;
    // For later usage
    sensors.push_back(sample.sensor);
  }
  // EKF update
  Eigen::MatrixXd H, R, S, K, oldP, newP;
  Eigen::VectorXd oldX, newX, diffZ;
  oldX.segment<3>(0) = position_;
  oldX.segment<3>(3) = velocity_;
  oldX.segment<4>(6) = Eigen::Vector4d(
    rotation_.w(),
    rotation_.x(),
    rotation_.y(),
    rotation_.z());
  oldP = covariance_;
  R = SliceR(measure_covariance_, sensors);
  // Choose the model according to the orientation
  if (msg.axis == HORIZONTAL) {
    H = GetHorizontalH(position_, rotation_, photodiodes);
    diffZ = Z - GetHorizontalZ(position_, rotation_, photodiodes);
  } else {
    H = GetVerticalH(position_, rotation_, photodiodes);
    diffZ = Z - GetVerticalZ(position_, rotation_, photodiodes);
  }
  S = H * oldP * H.transpose() * R;
  K = oldP * H.transpose() * S.inverse();
  newX = oldX + K * diffZ;
  newP = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * oldP;

  // Save new state
  position_ = newX.segment<3>(0);
  velocity_ = newX.segment<3>(3);
  rotation_ = Eigen::Quaterniond(newX(6),
    newX(7),
    newX(8),
    newX(9)).normalized();
  covariance_ = newP;
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