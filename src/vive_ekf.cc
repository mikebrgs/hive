// ROS includes
#include <hive/vive_ekf.h>

#define STATE_SIZE 13
#define NOISE_SIZE 9
#define SENSORS_SIZE 5
#define DT 0.01

namespace ekf {
  Eigen::Vector3d ConvertMessage(geometry_msgs::Vector3 msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
  }

  Eigen::Vector3d ConvertMessage(geometry_msgs::Point msg) {
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
}

ViveEKF::ViveEKF() {
  // Position initialization
  position_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  // Velocity initialization
  velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Quaternion initialization
  rotation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  // Bias initialization
  bias_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Model covariance
  model_covariance_ = 1e1 * Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE);
  // Measure covariance - assuming 3 sensors for now
  measure_covariance_ = 1e-1 * Eigen::MatrixXd::Identity(2 * SENSORS_SIZE,
    2 * SENSORS_SIZE);
  // Aux variables
  covariance_ = model_covariance_;
  // Virtual tracker
  tracker_.serial = "virtual";
  geometry_msgs::Point sensor0;
  sensor0.x = 0.1;
  sensor0.y = 0.1;
  sensor0.z = 0.0;
  geometry_msgs::Point sensor1;
  sensor1.x = -0.1;
  sensor1.y = -0.1;
  sensor1.z = 0.0;
  geometry_msgs::Point sensor2;
  sensor2.x = 0.1;
  sensor2.y = -0.1;
  sensor2.z = 0.0;
  geometry_msgs::Point sensor3;
  sensor3.x = -0.1;
  sensor3.y = 0.1;
  sensor3.z = 0.0;
  geometry_msgs::Point sensor4;
  sensor4.x = 0.0;
  sensor4.y = 0.0;
  sensor4.z = 0.1;
  tracker_.sensors[0].position = sensor0;
  tracker_.sensors[1].position = sensor1;
  tracker_.sensors[2].position = sensor2;
  tracker_.sensors[3].position = sensor3;
  tracker_.sensors[4].position = sensor4;
  // Tracker frame transform -- head to light
  tracker_.head_transform.translation.x = 0.0;
  tracker_.head_transform.translation.y = 0.0;
  tracker_.head_transform.translation.z = 0.0;
  tracker_.head_transform.rotation.w = 1.0;
  tracker_.head_transform.rotation.x = 0.0;
  tracker_.head_transform.rotation.y = 0.0;
  tracker_.head_transform.rotation.z = 0.0;
  // Tracker frame transform -- head to light
  tracker_.imu_transform.translation.x = 0.0;
  tracker_.imu_transform.translation.y = 0.0;
  tracker_.imu_transform.translation.z = 0.0;
  tracker_.imu_transform.rotation.w = 1.0;
  tracker_.imu_transform.rotation.x = 0.0;
  tracker_.imu_transform.rotation.y = 0.0;
  tracker_.imu_transform.rotation.z = 0.0;
  return;
}

ViveEKF::ViveEKF(geometry_msgs::TransformStamped & pose,
  Tracker & tracker,
  std::map<std::string, Lighthouse> & lighthouses,
  Environment & environment,
  double ** model_covariance, // time update
  double ** measure_covariance, // measurements
  bool correction) {
  position_ = ekf::ConvertMessage(pose.transform.translation);
  rotation_ = ekf::ConvertMessage(pose.transform.rotation);
  velocity_ = Eigen::Vector3d::Zero();
  bias_ = Eigen::Vector3d::Zero();
  tracker_ = tracker;
  lighthouses_ = lighthouses;
  environment_ = environment;
  correction_ = correction;
  model_covariance_ = Eigen::MatrixXd(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; i++) {
    for (size_t j = 0; j < STATE_SIZE; j++) {
      model_covariance_(i,j) = model_covariance[i][j];
    }
  }
  measure_covariance_ = Eigen::MatrixXd(2*tracker_.sensors.size(),
    2*tracker_.sensors.size());
  for (size_t i = 0; i < 2*tracker_.sensors.size(); i++) {
    for (size_t j = 0; j < 2*tracker_.sensors.size(); j++) {
      measure_covariance_(i,j) = measure_covariance[i][j];
    }
  }
  return;
}

ViveEKF::~ViveEKF() {
  return;
}

void ViveEKF::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  Predict(*msg);
  return;
}

void ViveEKF::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }
  Update(*msg);
  return;
}

bool ViveEKF::GetTransform(geometry_msgs::TransformStamped& msg) {
  // Change to this to be in the light frame
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

  double vQt_w = rotation.w();
  double vQt_x = rotation.x();
  double vQt_y = rotation.y();
  double vQt_z = rotation.z();

  double tA_x = linear_acceleration(0);
  double tA_y = linear_acceleration(1);
  double tA_z = linear_acceleration(2);

  double tW_x = angular_velocity(0);
  double tW_y = angular_velocity(1);
  double tW_z = angular_velocity(2);

  double tB_x = bias(0);
  double tB_y = bias(1);
  double tB_z = bias(2);

  // Set matrix
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;
  F.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  F.block<3,4>(0,6) = Eigen::MatrixXd::Zero(3,4);
  F.block<3,3>(0,10) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(3,0) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(3,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(3,10) = Eigen::MatrixXd::Zero(3,3);
  F.block<4,3>(6,0) = Eigen::MatrixXd::Zero(4,3);
  F.block<4,3>(6,3) = Eigen::MatrixXd::Zero(4,3);
  F.block<4,3>(6,10) = -0.5 * ekf::GetOmega(rotation);
  F.block<3,3>(10,0) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(10,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,4>(10,6) = Eigen::MatrixXd::Zero(3,4);
  F.block<3,3>(10,10) = Eigen::MatrixXd::Zero(3,3);

  // new d\dot{V}/d_qw
  F(3,6) = tA_y*vQt_z*-2.0+tA_z*vQt_y*2.0;
  F(4,6) = tA_x*vQt_z*2.0-tA_z*vQt_x*2.0;
  F(5,6) = tA_x*vQt_y*-2.0+tA_y*vQt_x*2.0;
  // new d\dot{V}/d_qx
  F(3,7) = tA_y*vQt_y*2.0+tA_z*vQt_z*2.0;
  F(4,7) = tA_x*vQt_y*2.0-tA_y*vQt_x*4.0-tA_z*vQt_w*2.0;
  F(5,7) = tA_y*vQt_w*2.0+tA_x*vQt_z*2.0-tA_z*vQt_x*4.0;
  // new d\dot{V}/d_qy
  F(3,8) = tA_x*vQt_y*-4.0+tA_y*vQt_x*2.0+tA_z*vQt_w*2.0;
  F(4,8) = tA_x*vQt_x*2.0+tA_z*vQt_z*2.0;
  F(5,8) = tA_x*vQt_w*-2.0+tA_y*vQt_z*2.0-tA_z*vQt_y*4.0;
  // new d\dot{V}/d_qz
  F(3,9) = tA_y*vQt_w*-2.0-tA_x*vQt_z*4.0+tA_z*vQt_x*2.0;
  F(3,9) = tA_x*vQt_w*2.0-tA_y*vQt_z*4.0+tA_z*vQt_y*2.0;
  F(3,9) = tA_x*vQt_x*2.0+tA_y*vQt_y*2.0;

  // d\dot{d}/dq_w
  F(6,6) = 0.0;
  F(7,6) = tB_x*(-1.0/2.0)+tW_x*(1.0/2.0);
  F(8,6) = tB_y*(-1.0/2.0)+tW_y*(1.0/2.0);
  F(9,6) = tB_z*(-1.0/2.0)+tW_z*(1.0/2.0);
  // d\dot{d}/dq_x
  F(6,7) = tB_x*(1.0/2.0)-tW_x*(1.0/2.0);
  F(7,7) = 0.0;
  F(8,7) = tB_z*(1.0/2.0)-tW_z*(1.0/2.0);
  F(9,7) = tB_y*(-1.0/2.0)+tW_y*(1.0/2.0);
  // d\dot{d}/dq_y
  F(6,8) = tB_y*(1.0/2.0)-tW_y*(1.0/2.0);
  F(7,8) = tB_z*(-1.0/2.0)+tW_z*(1.0/2.0);
  F(8,8) = 0.0;
  F(9,8) = tB_x*(1.0/2.0)-tW_x*(1.0/2.0);
  // d\dot{d}/dq_z
  F(6,9) = tB_z*(1.0/2.0)-tW_z*(1.0/2.0);
  F(7,9) = tB_y*(1.0/2.0)-tW_y*(1.0/2.0);
  F(8,9) = tB_x*(-1.0/2.0)+tW_x*(1.0/2.0);
  F(9,9) = 0.0;

  return F;
}

// Derivative of the measurement model - Horizontal measures
Eigen::MatrixXd GetHorizontalH(Eigen::Vector3d translation,
  Eigen::Quaterniond rotation,
  std::vector<int> sensors,
  Tracker tracker,
  Transform lhTransform,
  Lighthouse lhSpecs,
  bool correction) {
  Eigen::MatrixXd H = Eigen::MatrixXd(sensors.size(), STATE_SIZE);
  // Position of the tracker in the vive frame
  double vPt_x = translation(0);
  double vPt_y = translation(1);
  double vPt_z = translation(2);
  // Orientation of the tracker in the vive frame
  double vQt_w = rotation.w();
  double vQt_x = rotation.x();
  double vQt_y = rotation.y();
  double vQt_z = rotation.z();
  // Get transform from light frame to imu frame
  Eigen::Vector3d light_P_imu = ekf::ConvertMessage(
    tracker.imu_transform.translation);
  Eigen::Quaterniond light_Q_imu = ekf::ConvertMessage(
    tracker.imu_transform.rotation);
  Eigen::Vector3d imu_P_light =
    - light_Q_imu.toRotationMatrix().transpose() * light_P_imu;
  Eigen::Matrix3d imu_R_light = light_Q_imu.toRotationMatrix().transpose();
  // Position of the tracker's light frame in the IMU frame (default)
  double tPtl_x = imu_P_light(0);
  double tPtl_y = imu_P_light(1);
  double tPtl_z = imu_P_light(2);
  // Orientation of the tracker's light frame in the IMU frame (default)
  double tRtl_11 = imu_R_light(0,0);
  double tRtl_12 = imu_R_light(0,1);
  double tRtl_13 = imu_R_light(0,2);
  double tRtl_21 = imu_R_light(1,0);
  double tRtl_22 = imu_R_light(1,1);
  double tRtl_23 = imu_R_light(1,2);
  double tRtl_31 = imu_R_light(2,0);
  double tRtl_32 = imu_R_light(2,1);
  double tRtl_33 = imu_R_light(2,2);
  // Convert lighthouse orientation
  Eigen::Matrix3d vRl = ekf::ConvertMessage(
    lhTransform.rotation).toRotationMatrix();
  // Position of the lighthouse in vive
  double vPl_x = lhTransform.translation.x;
  double vPl_y = lhTransform.translation.y;
  double vPl_z = lhTransform.translation.z;
  // Orientation of the lighthouse in vive
  double vRl_11 = vRl(0,0);
  double vRl_12 = vRl(0,1);
  double vRl_13 = vRl(0,2);
  double vRl_21 = vRl(1,0);
  double vRl_22 = vRl(1,1);
  double vRl_23 = vRl(1,2);
  double vRl_31 = vRl(2,0);
  double vRl_32 = vRl(2,1);
  double vRl_33 = vRl(2,2);

  size_t row = 0;
  // Iterate over all photodiodes
  for (auto sensor : sensors) {
    // Position of the photodiode in the tracker's light frame
    double tlPs_x = tracker.sensors[sensor].position.x;
    double tlPs_y = tracker.sensors[sensor].position.y;
    double tlPs_z = tracker.sensors[sensor].position.z;
    // d alpha / d Px
    H(row, 0) = -(vRl_11/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_13*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Py
    H(row, 1) = -(vRl_21/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_23*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Pz
    H(row, 2) = -(vRl_31/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_33*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d V
    H(row, 3) = 0.0;
    H(row, 4) = 0.0;
    H(row, 5) = 0.0;
    // d alpha / d Qw
    H(row, 6) = -(((vQt_z*vRl_21*2.0-vQt_y*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_z*vRl_11*2.0-vQt_x*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vQt_y*vRl_11*2.0-vQt_x*vRl_21*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_z*vRl_23*2.0-vQt_y*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_z*vRl_13*2.0-vQt_x*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vQt_y*vRl_13*2.0-vQt_x*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qx
    H(row, 7) = -(((vQt_y*vRl_21*2.0+vQt_z*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_y*vRl_11*2.0-vQt_x*vRl_21*4.0+vQt_w*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_z*vRl_11*-2.0+vQt_w*vRl_21*2.0+vQt_x*vRl_31*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_y*vRl_23*2.0+vQt_z*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_y*vRl_13*2.0-vQt_x*vRl_23*4.0+vQt_w*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_z*vRl_13*-2.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qy
    H(row, 8) = -(((vQt_x*vRl_11*2.0+vQt_z*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_y*vRl_11*4.0-vQt_x*vRl_21*2.0+vQt_w*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_w*vRl_11*2.0+vQt_z*vRl_21*2.0-vQt_y*vRl_31*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_z*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_y*vRl_13*4.0-vQt_x*vRl_23*2.0+vQt_w*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_w*vRl_13*2.0+vQt_z*vRl_23*2.0-vQt_y*vRl_33*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qz
    H(row, 9) = -(((vQt_x*vRl_11*2.0+vQt_y*vRl_21*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_11*-4.0+vQt_w*vRl_21*2.0+vQt_x*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_11*2.0+vQt_z*vRl_21*4.0-vQt_y*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // row
    row++;
  }
  return H;
}

// Derivative of the measurement model - Vertical measures
Eigen::MatrixXd GetVerticalH(Eigen::Vector3d translation,
  Eigen::Quaterniond rotation,
  std::vector<int> sensors,
  Tracker tracker,
  Transform lhTransform,
  Lighthouse lhSpecs,
  bool correction) {
  Eigen::MatrixXd H = Eigen::MatrixXd(sensors.size(), STATE_SIZE);
  // Position of the tracker in the vive frame
  double vPt_x = translation(0);
  double vPt_y = translation(1);
  double vPt_z = translation(2);
  // Orientation of the tracker in the vive frame
  double vQt_w = rotation.w();
  double vQt_x = rotation.x();
  double vQt_y = rotation.y();
  double vQt_z = rotation.z();
  // Get transform from light frame to imu frame
  Eigen::Vector3d light_P_imu = ekf::ConvertMessage(
    tracker.imu_transform.translation);
  Eigen::Quaterniond light_Q_imu = ekf::ConvertMessage(
    tracker.imu_transform.rotation);
  Eigen::Vector3d imu_P_light =
    - light_Q_imu.toRotationMatrix().transpose() * light_P_imu;
  Eigen::Matrix3d imu_R_light = light_Q_imu.toRotationMatrix().transpose();
  // Position of the tracker's light frame in the IMU frame (default)
  double tPtl_x = imu_P_light(0);
  double tPtl_y = imu_P_light(1);
  double tPtl_z = imu_P_light(2);
  // Orientation of the tracker's light frame in the IMU frame (default)
  double tRtl_11 = imu_R_light(0,0);
  double tRtl_12 = imu_R_light(0,1);
  double tRtl_13 = imu_R_light(0,2);
  double tRtl_21 = imu_R_light(1,0);
  double tRtl_22 = imu_R_light(1,1);
  double tRtl_23 = imu_R_light(1,2);
  double tRtl_31 = imu_R_light(2,0);
  double tRtl_32 = imu_R_light(2,1);
  double tRtl_33 = imu_R_light(2,2);
  // Convert lighthouse orientation
  Eigen::Matrix3d vRl = ekf::ConvertMessage(
    lhTransform.rotation).toRotationMatrix();
  // Position of the lighthouse in vive
  double vPl_x = lhTransform.translation.x;
  double vPl_y = lhTransform.translation.y;
  double vPl_z = lhTransform.translation.z;
  // Orientation of the lighthouse in vive
  double vRl_11 = vRl(0,0);
  double vRl_12 = vRl(0,1);
  double vRl_13 = vRl(0,2);
  double vRl_21 = vRl(1,0);
  double vRl_22 = vRl(1,1);
  double vRl_23 = vRl(1,2);
  double vRl_31 = vRl(2,0);
  double vRl_32 = vRl(2,1);
  double vRl_33 = vRl(2,2);

  size_t row = 0;
  // Iterate over all photodiodes
  for (auto sensor : sensors) {
    // Position of the photodiode in the tracker's light frame
    double tlPs_x = tracker.sensors[sensor].position.x;
    double tlPs_y = tracker.sensors[sensor].position.y;
    double tlPs_z = tracker.sensors[sensor].position.z;
    // d alpha / d Px
    H(row, 0) = -(vRl_12/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_13*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Py
    H(row, 1) = -(vRl_22/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_23*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Pz
    H(row, 2) = -(vRl_32/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-vRl_33*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d V
    H(row, 3) = 0.0;
    H(row, 4) = 0.0;
    H(row, 5) = 0.0;
    // d alpha / d Qw
    H(row, 6) = -(((vQt_z*vRl_22*2.0-vQt_y*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_z*vRl_12*2.0-vQt_x*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vQt_y*vRl_12*2.0-vQt_x*vRl_22*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_z*vRl_23*2.0-vQt_y*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_z*vRl_13*2.0-vQt_x*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vQt_y*vRl_13*2.0-vQt_x*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qx
    H(row, 7) = -(((vQt_y*vRl_22*2.0+vQt_z*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_y*vRl_12*2.0-vQt_x*vRl_22*4.0+vQt_w*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_z*vRl_12*-2.0+vQt_w*vRl_22*2.0+vQt_x*vRl_32*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_y*vRl_23*2.0+vQt_z*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_y*vRl_13*2.0-vQt_x*vRl_23*4.0+vQt_w*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_z*vRl_13*-2.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qy
    H(row, 8) = -(((vQt_x*vRl_12*2.0+vQt_z*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_y*vRl_12*4.0-vQt_x*vRl_22*2.0+vQt_w*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_w*vRl_12*2.0+vQt_z*vRl_22*2.0-vQt_y*vRl_32*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_z*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)-(vQt_y*vRl_13*4.0-vQt_x*vRl_23*2.0+vQt_w*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vQt_w*vRl_13*2.0+vQt_z*vRl_23*2.0-vQt_y*vRl_33*4.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // d alpha / d Qz
    H(row, 9) = -(((vQt_x*vRl_12*2.0+vQt_y*vRl_22*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_12*-4.0+vQt_w*vRl_22*2.0+vQt_x*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_12*2.0+vQt_z*vRl_22*4.0-vQt_y*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0);
    // row
    row++;
  }
  return H;
}

Eigen::MatrixXd GetG(Eigen::Quaterniond rotation) {
  Eigen::MatrixXd G = Eigen::MatrixXd(STATE_SIZE, NOISE_SIZE);
  G.block<3,9>(0,0) = Eigen::MatrixXd::Zero(3,9);
  G.block<3,3>(3,0) = Eigen::MatrixXd::Zero(3,3);
  G.block<3,3>(3,3) = rotation.toRotationMatrix();
  G.block<4,3>(6,0) = ekf::GetOmega(rotation);
  G.block<4,3>(6,3) = Eigen::MatrixXd::Zero(4,3);
  G.block<7,3>(3,6) = Eigen::MatrixXd::Zero(7,3);
  G.block<3,6>(10,0) = Eigen::MatrixXd::Zero(3,6);
  G.block<3,3>(10,6) = Eigen::MatrixXd::Identity(3,3);
  return G;
}

// Vector that represents the evolution of the system is continuous time
Eigen::MatrixXd GetDState(Eigen::Vector3d velocity,
  Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity,
  Eigen::Vector3d gravity,
  Eigen::Vector3d bias) {
  Eigen::VectorXd dotX = Eigen::VectorXd(STATE_SIZE);
  // Position
  dotX.segment<3>(0) = velocity;
  // Velocity - assumed constant
  dotX.segment<3>(3) = rotation.toRotationMatrix() * linear_acceleration + gravity;
  // Orientation
  Eigen::MatrixXd Omega = ekf::GetOmega(rotation);
  dotX.segment<4>(6) = 0.5 * (Omega * angular_velocity - bias);
  dotX.segment<3>(10) = Eigen::VectorXd::Zero(3);
  return dotX;
}

// Vector that represents the horizontal predicted measurements according to the
// system's state
Eigen::MatrixXd GetHorizontalZ(Eigen::Vector3d position,
  Eigen::Quaterniond rotation,
  std::vector<int> sensors,
  Tracker tracker,
  Transform lhTransform,
  Lighthouse lhSpecs,
  bool correction) {
  // Declaring measurements vector
  Eigen::VectorXd Z = Eigen::VectorXd(sensors.size());

  // Data conversion
  Eigen::Vector3d vPt = position;
  Eigen::Matrix3d vRt = rotation.toRotationMatrix();
  Eigen::Vector3d vPl = ekf::ConvertMessage(lhTransform.translation);
  Eigen::Matrix3d vRl = ekf::ConvertMessage(lhTransform.rotation).toRotationMatrix();
  Eigen::Vector3d lPv = - vRl.transpose() * vPl;
  Eigen::Matrix3d lRv = vRl.transpose();
  // lightPimu - transform the imu frame to the light frame
  Eigen::Vector3d tlPt = ekf::ConvertMessage(
    tracker.imu_transform.translation);
  // lightRimu - transform the imu frame to the light frame
  Eigen::Matrix3d tlRt = ekf::ConvertMessage(
    tracker.imu_transform.rotation).toRotationMatrix();
  // conversion from tlTt to tTtl
  Eigen::Vector3d tPtl = - tlRt.transpose() * tlPt;
  Eigen::Matrix3d tRtl = tlRt.transpose();

  size_t row = 0;
  for (auto sensor : sensors) {
    Eigen::Vector3d tlPs = ekf::ConvertMessage(tracker.sensors[sensor].position);
    Eigen::Vector3d lPs = lRv * ( vRt * (tRtl * tlPs + tPtl) + vPt ) + lPv;
    Z(row) = atan2(lPs(0),lPs(2));
    row++;
  }
  return Z;
}

// Vector that represents the vertical predicted measurements according to the
// system's state
Eigen::MatrixXd GetVerticalZ(Eigen::Vector3d position,
  Eigen::Quaterniond rotation,
  std::vector<int> sensors,
  Tracker tracker,
  Transform lhTransform,
  Lighthouse lhSpecs,
  bool correction) {
  // Declaring measurements vector
  Eigen::VectorXd Z = Eigen::VectorXd(sensors.size());

  // Data conversion
  Eigen::Vector3d vPt = position;
  Eigen::Matrix3d vRt = rotation.toRotationMatrix();
  Eigen::Vector3d vPl = ekf::ConvertMessage(lhTransform.translation);
  Eigen::Matrix3d vRl = ekf::ConvertMessage(lhTransform.rotation).toRotationMatrix();
  Eigen::Vector3d lPv = - vRl.transpose() * vPl;
  Eigen::Matrix3d lRv = vRl.transpose();
  // lightPimu - transform the imu frame to the light frame
  Eigen::Vector3d tlPt = ekf::ConvertMessage(
    tracker.imu_transform.translation);
  // lightRimu - transform the imu frame to the light frame
  Eigen::Matrix3d tlRt = ekf::ConvertMessage(
    tracker.imu_transform.rotation).toRotationMatrix();
  // conversion from tlTt to tTtl
  Eigen::Vector3d tPtl = - tlRt.transpose() * tlPt;
  Eigen::Matrix3d tRtl = tlRt.transpose();

  size_t row = 0;
  for (auto sensor : sensors) {
    Eigen::Vector3d tlPs = ekf::ConvertMessage(tracker.sensors[sensor].position);
    Eigen::Vector3d lPs = lRv * ( vRt * (tRtl * tlPs + tPtl) + vPt ) + lPv;
    Z(row) = atan2(lPs(1),lPs(2));
    row++;
  }
  return Z;
}

Eigen::MatrixXd SliceR(Eigen::MatrixXd R,
  std::vector<int> sensors) {
  size_t col = 0, row = 0;
  Eigen::MatrixXd slicedR = Eigen::MatrixXd(sensors.size(),
    sensors.size());
  for (auto sensor_row : sensors) {
    col = 0;
    for (auto sensor_col : sensors) {
      slicedR(row, col) = R(sensor_row, sensor_col);
      col++;
    }
    row++;
  }
  return slicedR;
}

// Time update (Inertial data)
bool ViveEKF::Predict(const sensor_msgs::Imu & msg) {
  // Convert measurements
  Eigen::Vector3d linear_acceleration = ekf::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d angular_velocity = ekf::ConvertMessage(msg.angular_velocity);
  Eigen::Vector3d gravity = ekf::ConvertMessage(environment_.gravity);
  // Time difference TODO change this
  double dT = DT;//(msg.header.stamp - time_).toSec();

  // Derivative of the state in time
  Eigen::MatrixXd dState = GetDState(velocity_,
    rotation_,
    linear_acceleration,
    angular_velocity,
    gravity,
    bias_);
  // Old state
  Eigen::Matrix<double, STATE_SIZE, 1> oldX;
  oldX.block<3,1>(0,0) = position_;
  oldX.block<3,1>(3,0) = velocity_;
  oldX.block<4,1>(6,0) = Eigen::Vector4d(
    rotation_.w(),
    rotation_.x(),
    rotation_.y(),
    rotation_.z());

  // Covariance update
  Eigen::MatrixXd oldP = covariance_;
  Eigen::MatrixXd F = GetF(rotation_,
    linear_acceleration,
    angular_velocity,
    bias_);
  Eigen::MatrixXd G = GetG(rotation_);
  Eigen::MatrixXd Q = model_covariance_;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE,
    STATE_SIZE);
  // std::cout << "F: " << F << std::endl;
  // std::cout << "Q: " << Q << std::endl;
  // std::cout << "oldP: " << oldP << std::endl;
  Eigen::MatrixXd newP = (I + dT * F) *
  oldP * (I + dT * F).transpose() + dT * dT * G * Q * G.transpose();

  // New state
  Eigen::MatrixXd newX = oldX + dT * dState;
  position_ = newX.block<3,1>(0,0);
  velocity_ = newX.block<3,1>(3,0);
  rotation_ = Eigen::Quaterniond(newX(6), newX(7), newX(8),
    newX(9)).normalized();
  covariance_ = newP;
  time_ = msg.header.stamp;
  return true;
}

// Measure upate (Light data)
bool ViveEKF::Update(const hive::ViveLight & msg) {
  size_t row = 0;
  std::vector<Eigen::Vector3d> photodiodes;
  Eigen::MatrixXd Z = Eigen::VectorXd(msg.samples.size());
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
  Eigen::MatrixXd H, R, K, oldP, newP;
  Eigen::VectorXd oldX = Eigen::VectorXd(STATE_SIZE), newX, Y;
  oldX.segment<3>(0) = position_;
  oldX.segment<3>(3) = velocity_;
  oldX.segment<4>(6) = Eigen::Vector4d(
    rotation_.w(),
    rotation_.x(),
    rotation_.y(),
    rotation_.z());
  oldP = covariance_;

  // Choose the model according to the orientation
  if (msg.axis == HORIZONTAL) {
    // Derivative of measurement model
    H = GetHorizontalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[msg.lighthouse],
      lighthouses_[msg.lighthouse], correction_);
    // Difference between prediction and real measures
    Y = Z - GetHorizontalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[msg.lighthouse],
      lighthouses_[msg.lighthouse], correction_);
    // Sliced measurement covariance matrix
    R = SliceR(measure_covariance_.block<SENSORS_SIZE,
      SENSORS_SIZE>(SENSORS_SIZE*HORIZONTAL,
      SENSORS_SIZE*HORIZONTAL), sensors);
  } else {
    // Derivative of measurement model
    H = GetVerticalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[msg.lighthouse],
      lighthouses_[msg.lighthouse], correction_);
    // Difference between prediction and real measures
    Y = Z - GetVerticalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[msg.lighthouse],
      lighthouses_[msg.lighthouse], correction_);
    // Sliced measurement covariance matrix
    R = SliceR(measure_covariance_.block<SENSORS_SIZE,
      SENSORS_SIZE>(SENSORS_SIZE*VERTICAL,
      SENSORS_SIZE*VERTICAL), sensors);
  }

  // Kalman Gain
  K = oldP * H.transpose() * (H * oldP * H.transpose() + R).inverse();
  newX = oldX + K * Y;
  newP = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * oldP;

  // Save new state
  position_ = newX.segment<3>(0);
  velocity_ = newX.segment<3>(3);
  rotation_ = Eigen::Quaterniond(newX(6),
    newX(7),
    newX(8),
    newX(9)).normalized();
  covariance_ = newP;
  time_ = msg.header.stamp;

  return true;
}

void ViveEKF::PrintState() {
  std::cout << "Position: "
    << position_(0) << ", "
    << position_(1) << ", "
    << position_(2) << std::endl;
  std::cout << "Velocity: "
    << velocity_(0) << ", "
    << velocity_(1) << ", "
    << velocity_(2) << std::endl;
  std::cout << "Orientation: "
    << rotation_.w() << ", "
    << rotation_.x() << ", "
    << rotation_.y() << ", "
    << rotation_.z() << std::endl;
  std::cout << "Bias: "
    << bias_(0) << ", "
    << bias_(1) << ", "
    << bias_(2) << std::endl;
  std::cout << "Covariance: "
    << covariance_ << std::endl;
  return;
}

int main(int argc, char ** argv)
{
  ROS_INFO("FILTERING");

  ViveEKF filter = ViveEKF();
  ViveModel model = ViveModel();
  // Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);
  // Eigen::Vector3d angular_velocity(0.0, 0.0, 0.0);
  // model.Update(linear_acceleration, angular_velocity, 0.1);
  // return 0;
  std::cout << "***MODEL***" << std::endl;
  model.PrintState();
  std::cout << "***FILTER***" << std::endl;
  filter.PrintState();
  for (size_t i = 0; i < 40; i++) {
    { // Inertial message
      Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);
      Eigen::Vector3d angular_velocity(0.0, 0.0, 1.0);
      model.Update(linear_acceleration, angular_velocity, DT);
    }
    { // Inertial update
      std::cout << "INERTIAL" << std::endl;
      Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);
      Eigen::Vector3d angular_velocity(0.0, 0.0, 0.0);
      sensor_msgs::Imu msg;
      msg = model.GetInertialMeasures();
      // Prediction based on inertial data
      filter.Predict(msg);
      std::cout << "***MODEL***" << std::endl;
      model.PrintState();
      std::cout << "***FILTER***" << std::endl;
      filter.PrintState();
    }
    { // Light update
      std::cout << "LIGHT" << std::endl;
      hive::ViveLight msg;
      if (i % 2 == 0)
        msg = model.GetHorizontalLightMeasures();
      else
        msg = model.GetVerticalLightMeasures();
      // Prediction based on inertial data
      filter.Update(msg);
      std::cout << "***MODEL***" << std::endl;
      model.PrintState();
      std::cout << "***FILTER***" << std::endl;
      filter.PrintState();
    }
  }
  return 0;
}