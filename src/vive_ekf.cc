// ROS includes
#include <hive/vive_ekf.h>
#include <hive/vive_ekf_diff.h>

#define STATE_SIZE 13
#define NOISE_SIZE 9
#define SENSORS_SIZE 5
#define DT 0.005

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
  model_covariance_ = 1e1 * Eigen::MatrixXd::Identity(NOISE_SIZE,NOISE_SIZE);
  // Measure covariance - assuming 3 sensors for now
  measure_covariance_ = 1e-4 * Eigen::MatrixXd::Identity(2 * SENSORS_SIZE,
    2 * SENSORS_SIZE);
  // Aux variables
  covariance_ = 1e0 * Eigen::MatrixXd::Identity(STATE_SIZE,STATE_SIZE);
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
  // Environment
  Transform lh1;
  lh1.translation.x = 0.0;
  lh1.translation.y = 0.0;
  lh1.translation.z = 0.0;
  lh1.rotation.w = 1.0;
  lh1.rotation.x = 0.0;
  lh1.rotation.y = 0.0;
  lh1.rotation.z = 0.0;
  lh1.parent_frame = "vive";
  lh1.child_frame = "lh1";
  environment_.lighthouses["lh1"] = lh1;
  // Gravity
  environment_.gravity.x = 0.0;
  environment_.gravity.y = 0.0;
  environment_.gravity.z = 0.0;

  valid_ = true;
  return;
}

ViveEKF::ViveEKF(Tracker & tracker,
  std::map<std::string, Lighthouse> & lighthouses,
  Environment & environment,
  double model_noise, // time update
  double measure_noise, // measurements
  bool correction) {
  position_ = Eigen::Vector3d::Zero();
  rotation_ = Eigen::Quaterniond::Identity();
  velocity_ = Eigen::Vector3d::Zero();
  bias_ = Eigen::Vector3d::Zero();
  tracker_ = tracker;
  lighthouses_ = lighthouses;
  environment_ = environment;
  valid_ = false;
  correction_ = correction;
  model_covariance_ = model_noise *
    Eigen::MatrixXd::Identity(NOISE_SIZE, NOISE_SIZE);
  measure_covariance_ = measure_noise * Eigen::MatrixXd::Identity(
    2*tracker_.sensors.size(), 2*tracker_.sensors.size());
  covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  return;
}


ViveEKF::~ViveEKF() {
  return;
}

void ViveEKF::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL || !valid_) {
    return;
  }
  Predict(*msg);
  return;
}

void ViveEKF::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return;

  if (msg->axis == HORIZONTAL)
    ldata_[msg->lighthouse].first = msg;
  else if (msg->axis == VERTICAL)
    ldata_[msg->lighthouse].second = msg;

  if (!valid_) {
    if (ldata_[msg->lighthouse].first == NULL
      || ldata_[msg->lighthouse].second == NULL) return;
    // Solve rapidly

    double pose[9];
    pose[0] = 0.0;
    pose[1] = 0.0;
    pose[2] = 1.0;
    pose[3] = 0.0;
    pose[4] = 0.0;
    pose[5] = 0.0;
    pose[6] = 0.0;
    pose[7] = 0.0;
    pose[8] = 0.0;

    geometry_msgs::Transform lhTF;
    lhTF.translation.x = environment_.lighthouses[msg->lighthouse].translation.x;
    lhTF.translation.y = environment_.lighthouses[msg->lighthouse].translation.y;
    lhTF.translation.z = environment_.lighthouses[msg->lighthouse].translation.z;
    lhTF.rotation.w = environment_.lighthouses[msg->lighthouse].rotation.w;
    lhTF.rotation.x = environment_.lighthouses[msg->lighthouse].rotation.x;
    lhTF.rotation.y = environment_.lighthouses[msg->lighthouse].rotation.y;
    lhTF.rotation.z = environment_.lighthouses[msg->lighthouse].rotation.z;

    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    // Horizontal data
    ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
      new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
      (new ViveHorizontalCost(*ldata_[msg->lighthouse].first,
        lhTF,
        tracker_,
        lighthouses_[msg->lighthouse].horizontal_motor,
        correction_));
    hcost->AddParameterBlock(9);
    hcost->SetNumResiduals(ldata_[msg->lighthouse].first->samples.size());
    problem.AddResidualBlock(hcost, NULL, pose);
    // Vertical data
    ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
      new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
      (new ViveVerticalCost(*ldata_[msg->lighthouse].second,
        lhTF,
        tracker_,
        lighthouses_[msg->lighthouse].vertical_motor,
        correction_));
    vcost->AddParameterBlock(9);
    vcost->SetNumResiduals(ldata_[msg->lighthouse].second->samples.size());
    problem.AddResidualBlock(vcost, NULL, pose);

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 1000;
    ceres::Solve(options, &problem, &summary);

    position_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vAi(pose[6], pose[7], pose[8]);
    Eigen::AngleAxisd vAAi(vAi.norm(), vAi.normalized());
    rotation_ = Eigen::Quaterniond(vAAi);
    bias_ = Eigen::Vector3d(tracker_.gyr_bias.x,
      tracker_.gyr_bias.y,
      tracker_.gyr_bias.z);
    valid_ = true;
  } else {
    Update(*msg);
    valid_ = Valid();
  }
  return;
}

bool ViveEKF::Valid() {
  // check_validity
  // HERE
  return true;
}

bool ViveEKF::GetTransform(geometry_msgs::TransformStamped& msg) {
  if (!valid_) return false;
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
  msg.header.frame_id = "vive";
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
  Eigen::MatrixXd F(STATE_SIZE, STATE_SIZE);
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
  F(3,6) = tA_y*vQt_z*2.0-tA_z*vQt_y*2.0;
  F(4,6) = tA_x*vQt_z*-2.0+tA_z*vQt_x*2.0;
  F(5,6) = tA_x*vQt_y*2.0-tA_y*vQt_x*2.0;
  // new d\dot{V}/d_qx
  F(3,7) = tA_y*vQt_y*-2.0-tA_z*vQt_z*2.0;
  F(4,7) = tA_x*vQt_y*-2.0+tA_y*vQt_x*4.0+tA_z*vQt_w*2.0;
  F(5,7) = tA_y*vQt_w*-2.0-tA_x*vQt_z*2.0+tA_z*vQt_x*4.0;
  // new d\dot{V}/d_qy
  F(3,8) = tA_x*vQt_y*4.0-tA_y*vQt_x*2.0-tA_z*vQt_w*2.0;
  F(4,8) = tA_x*vQt_x*-2.0-tA_z*vQt_z*2.0;
  F(5,8) = tA_x*vQt_w*2.0-tA_y*vQt_z*2.0+tA_z*vQt_y*4.0;
  // new d\dot{V}/d_qz
  F(3,9) = tA_y*vQt_w*2.0+tA_x*vQt_z*4.0-tA_z*vQt_x*2.0;
  F(3,9) = tA_x*vQt_w*-2.0+tA_y*vQt_z*4.0-tA_z*vQt_y*2.0;
  F(3,9) = tA_x*vQt_x*-2.0-tA_y*vQt_y*2.0;

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
  Eigen::MatrixXd H(sensors.size(), STATE_SIZE);
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
  // Lighthouse parameters
  double phase = lhSpecs.horizontal_motor.phase;
  double tilt = lhSpecs.horizontal_motor.tilt;
  double gib_phase = lhSpecs.horizontal_motor.gib_phase;
  double gib_mag = lhSpecs.horizontal_motor.gib_magnitude;
  double curve = lhSpecs.horizontal_motor.curve;

  size_t row = 0;
  // Iterate over all photodiodes
  for (auto sensor : sensors) {
    // Position of the photodiode in the tracker's light frame
    double tlPs_x = tracker.sensors[sensor].position.x;
    double tlPs_y = tracker.sensors[sensor].position.y;
    double tlPs_z = tracker.sensors[sensor].position.z;

    H.block<13,1>(row,0) = HorizontalMeasureModelDiff(vPt_x, vPt_y, vPt_z,
      vQt_w, vQt_x, vQt_y, vQt_z,
      tPtl_x, tPtl_y, tPtl_z,
      tRtl_11, tRtl_12, tRtl_13,
      tRtl_21, tRtl_22, tRtl_23,
      tRtl_31, tRtl_32, tRtl_33,
      vPl_x, vPl_y, vPl_z,
      vRl_11, vRl_12, vRl_13,
      vRl_21, vRl_22, vRl_23,
      vRl_31, vRl_32, vRl_33,
      tlPs_x, tlPs_y, tlPs_z,
      phase, tilt, gib_phase, gib_mag, curve,
      correction);

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
  // Lighthouse parameters
  double phase = lhSpecs.vertical_motor.phase;
  double tilt = lhSpecs.vertical_motor.tilt;
  double gib_phase = lhSpecs.vertical_motor.gib_phase;
  double gib_mag = lhSpecs.vertical_motor.gib_magnitude;
  double curve = lhSpecs.vertical_motor.curve;

  size_t row = 0;
  // Iterate over all photodiodes
  for (auto sensor : sensors) {
    // Position of the photodiode in the tracker's light frame
    double tlPs_x = tracker.sensors[sensor].position.x;
    double tlPs_y = tracker.sensors[sensor].position.y;
    double tlPs_z = tracker.sensors[sensor].position.z;

    H.block<13,1>(row,0) = VerticalMeasureModelDiff(vPt_x, vPt_y, vPt_z,
      vQt_w, vQt_x, vQt_y, vQt_z,
      tPtl_x, tPtl_y, tPtl_z,
      tRtl_11, tRtl_12, tRtl_13,
      tRtl_21, tRtl_22, tRtl_23,
      tRtl_31, tRtl_32, tRtl_33,
      vPl_x, vPl_y, vPl_z,
      vRl_11, vRl_12, vRl_13,
      vRl_21, vRl_22, vRl_23,
      vRl_31, vRl_32, vRl_33,
      tlPs_x, tlPs_y, tlPs_z,
      phase, tilt, gib_phase, gib_mag, curve,
      correction);

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
Eigen::VectorXd GetDState(Eigen::Vector3d velocity,
  Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity,
  Eigen::Vector3d gravity,
  Eigen::Vector3d bias) {
  Eigen::VectorXd dotX(STATE_SIZE);
  // Position
  dotX.segment<3>(0) = velocity;
  // Velocity - assumed constant
  dotX.segment<3>(3) = - rotation.toRotationMatrix() * linear_acceleration + gravity;
  // Orientation
  Eigen::MatrixXd Omega = ekf::GetOmega(rotation);
  dotX.segment<4>(6) = 0.5 * (Omega * (angular_velocity - bias));
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
  // Lighthouse parameters
  double phase = lhSpecs.horizontal_motor.phase;
  double tilt = lhSpecs.horizontal_motor.tilt;
  double gib_phase = lhSpecs.horizontal_motor.gib_phase;
  double gib_mag = lhSpecs.horizontal_motor.gib_magnitude;
  double curve = lhSpecs.horizontal_motor.curve;

  size_t row = 0;
  for (auto sensor : sensors) {
    Eigen::Vector3d tlPs = ekf::ConvertMessage(tracker.sensors[sensor].position);
    Eigen::Vector3d lPs = lRv * ( vRt * (tRtl * tlPs + tPtl) + vPt ) + lPv;

    double x = (lPs(0)/lPs(2));
    double y = (lPs(1)/lPs(2));

    if (correction) {
      Z(row) = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
    } else {
      Z(row) = atan(x);
    }
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
  // Lighthouse parameters
  double phase = lhSpecs.vertical_motor.phase;
  double tilt = lhSpecs.vertical_motor.tilt;
  double gib_phase = lhSpecs.vertical_motor.gib_phase;
  double gib_mag = lhSpecs.vertical_motor.gib_magnitude;
  double curve = lhSpecs.vertical_motor.curve;

  size_t row = 0;
  for (auto sensor : sensors) {
    Eigen::Vector3d tlPs = ekf::ConvertMessage(tracker.sensors[sensor].position);
    Eigen::Vector3d lPs = lRv * ( vRt * (tRtl * tlPs + tPtl) + vPt ) + lPv;

    double x = (lPs(0)/lPs(2));
    double y = (lPs(1)/lPs(2));

    if (correction) {
      Z(row) = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
    } else {
      Z(row) = atan(y);
    }
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
  Eigen::VectorXd dState = GetDState(velocity_,
    rotation_,
    linear_acceleration,
    angular_velocity,
    gravity,
    bias_);
  // Old state
  Eigen::VectorXd oldX(STATE_SIZE);
  oldX.segment<3>(0) = position_;
  oldX.segment<3>(3) = velocity_;
  oldX.segment<4>(6) = Eigen::Vector4d(
    rotation_.w(),
    rotation_.x(),
    rotation_.y(),
    rotation_.z());
  oldX.segment<3>(10) = bias_;

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

  Eigen::MatrixXd newP = (I + dT * F) *
    oldP * (I + dT * F).transpose() + (dT * dT) * (G * Q * G.transpose()) + 1.0 * I;
    // This term increases the uncertainty

  // New state
  Eigen::VectorXd newX = oldX + dT * dState;
  position_ = newX.segment<3>(0);
  velocity_ = newX.segment<3>(3);
  rotation_ = Eigen::Quaterniond(newX(6), newX(7), newX(8),
    newX(9)).normalized();
  bias_ = newX.segment<3>(10);
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
  oldX.segment<3>(10) = bias_;
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
    // std::cout << "HORIZONTAL" << std::endl;
    // std::cout << "Z: " << Z.transpose() << std::endl;
    // std::cout << "Y: " << Y.transpose() << std::endl;
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
    // std::cout << "VERTICAL" << std::endl;
    // std::cout << "Z: " << Z.transpose() << std::endl;
    // std::cout << "Y: " << Y.transpose() << std::endl;
  }

  // Kalman Gain
  K = oldP * H.transpose() * (H * oldP * H.transpose() + R).inverse();
  newX = oldX + K * Y;
  newP = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * oldP;
  // std::cout << "K: " << K << std::endl;
  // std::cout << "oldP: " << oldP << std::endl;
  // std::cout << "H: " << H << std::endl;
  // std::cout << "R: " << R << std::endl;

  // Save new state
  position_ = newX.segment<3>(0);
  velocity_ = newX.segment<3>(3);
  rotation_ = Eigen::Quaterniond(newX(6),
    newX(7),
    newX(8),
    newX(9)).normalized();
  bias_ = newX.segment<3>(10);
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
    << covariance_.block<3,3>(0,0).trace() << ", "
    << covariance_.block<3,3>(3,3).trace() << ", "
    << covariance_.block<4,4>(6,6).trace() << ", "
    << covariance_.block<3,3>(10,10).trace() << std::endl;
  return;
}

int main(int argc, char ** argv) {
  ROS_INFO("FILTERING");

  std::map<std::string, ViveEKF> ekf_map;
  Calibration cal;
  rosbag::View view;
  rosbag::Bag rbag;

  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << std::endl;
    return -1;
  }

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
    ROS_FATAL("Can't find calibration file.");
    return -1;
  } else {
    ROS_INFO("Read calibration file.");
  }

  rbag.open(argv[1], rosbag::bagmode::Read);
  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal.SetTrackers(*vt);
    for (auto tr : vt->trackers) {
      // ekf_map[tr.serial] = ViveEKF(cal.environment,
      //   cal.trackers[tr.serial],
      //   cal.lighthouses,
      //   4, TRUST, true);
    }
  }
  ROS_INFO("Trackers' setup complete.");

  size_t counter = 0;
  // Light data
  std::vector<std::string> run_topics; 
  run_topics.push_back("/loc/vive/light");
  run_topics.push_back("/loc/vive/imu/");
  run_topics.push_back("/loc/vive/imu");
  rosbag::View view_li(rbag, rosbag::TopicQuery(run_topics));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    if (vl != NULL) {
      // std::cout << "LIGHT" << std::endl;
      ekf_map[vl->header.frame_id].ProcessLight(vl);
      counter++;
    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      // std::cout << "IMU" << std::endl;
      ekf_map[vi->header.frame_id].ProcessImu(vi);
      counter++;
    }
    // if (counter >= 200) break;
  }
  ROS_INFO("Data processment complete.");

  return 0;

}

  // ViveModel model = ViveModel();
  // std::cout << "***MODEL***" << std::endl;
  // model.PrintState();
  // std::cout << "***FILTER***" << std::endl;
  // filter.PrintState();

  // for (size_t i = 0; i < 80; i++) {
  //   { // Inertial message
  //     Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);
  //     Eigen::Vector3d angular_velocity(0.0, 0.0, 1.0);
  //     model.Update(linear_acceleration, angular_velocity, DT);
  //   }

  //     std::cout << "MODEL" << std::endl;
  //     model.PrintState();
  //   { // Inertial update
  //     sensor_msgs::Imu msg;
  //     msg = model.GetInertialMeasures();
  //     std::cout << "INERTIAL FILTER" << std::endl;
  //     // Prediction based on inertial data
  //     filter.Predict(msg);
  //     filter.PrintState();
  //   }

  //   { // Light update
  //     hive::ViveLight msg;
  //     if (i % 2 == 0) {
  //       msg = model.GetHorizontalLightMeasures();
  //     } else {
  //       msg = model.GetVerticalLightMeasures();
  //     }
  //     std::cout << "LIGHT FILTER" << std::endl;
  //     // Prediction based on light data
  //     filter.Update(msg);
  //     filter.PrintState();
  //   }
  //   std::cout << std::endl;
  // }
  // return 0;