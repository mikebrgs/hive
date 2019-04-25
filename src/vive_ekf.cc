// ROS includes
#include <hive/vive_ekf.h>
#include <hive/vive_ekf_diff.h>

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

ViveEKF::ViveEKF() {}

ViveEKF::ViveEKF(Tracker & tracker,
  std::map<std::string, Lighthouse> & lighthouses,
  Environment & environment,
  double model_noise, // time update
  double measure_noise, // measurements
  bool correction) {
  position_ = Eigen::Vector3d(0.0, 0.0, 1.0);
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
  // measure_covariance_ - double the size for both sweeps. The
  // first half of the matrix is for the horizontal, the second half
  // of the matrix is for the vertical sweep
  //    [ H 0 ]
  //    [ 0 V ]
  measure_covariance_ = measure_noise * Eigen::MatrixXd::Identity(
    2*tracker_.sensors.size(), 2*tracker_.sensors.size());
  covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  return;
}

ViveEKF::~ViveEKF() {
  return;
}

void ViveEKF::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) return;
  if (!valid_) return;

  Predict(*msg);

  lastmsgwasimu_ = true;

  return;
}

bool ViveEKF::Initialize(hive::ViveLight & msg) {
  return true;
}

void ViveEKF::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return;

  light_data_.push_back(*msg);
  if (light_data_.size() > LIGHT_DATA_BUFFER) {
    light_data_.erase(light_data_.begin());
  }

  if (!valid_ && light_data_.size() >= LIGHT_DATA_BUFFER) {
    // Solve rapidly

    double pose[9];
    pose[0] = position_(0);
    pose[1] = position_(1);
    pose[2] = position_(2);
    pose[3] = velocity_(0);
    pose[4] = velocity_(1);
    pose[5] = velocity_(2);
    Eigen::AngleAxisd pre_vAi(rotation_);
    pose[6] = pre_vAi.angle() * pre_vAi.axis()(0);
    pose[7] = pre_vAi.angle() * pre_vAi.axis()(1);
    pose[8] = pre_vAi.angle() * pre_vAi.axis()(2);


    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    for (auto sample : light_data_) {
      geometry_msgs::Transform lhTF;
      lhTF.translation.x = environment_.lighthouses[sample.lighthouse].translation.x;
      lhTF.translation.y = environment_.lighthouses[sample.lighthouse].translation.y;
      lhTF.translation.z = environment_.lighthouses[sample.lighthouse].translation.z;
      lhTF.rotation.w = environment_.lighthouses[sample.lighthouse].rotation.w;
      lhTF.rotation.x = environment_.lighthouses[sample.lighthouse].rotation.x;
      lhTF.rotation.y = environment_.lighthouses[sample.lighthouse].rotation.y;
      lhTF.rotation.z = environment_.lighthouses[sample.lighthouse].rotation.z;
      // Horizontal
      if (sample.axis == HORIZONTAL) {
        // Horizontal data
        ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
          (new ViveHorizontalCost(sample,
            lhTF,
            tracker_,
            lighthouses_[sample.lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(9);
        hcost->SetNumResiduals(sample.samples.size());
        problem.AddResidualBlock(hcost, NULL, pose);
      }
      // Vertical
      if (sample.axis == VERTICAL) {
        // Vertical data
        ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
          (new ViveVerticalCost(sample,
            lhTF,
            tracker_,
            lighthouses_[sample.lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(9);
        vcost->SetNumResiduals(sample.samples.size());
        problem.AddResidualBlock(vcost, NULL, pose);
      }
    }

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 1000;
    ceres::Solve(options, &problem, &summary);

    std::cout << "Init: " << summary.final_cost <<  " - "
      << pose[0] << ", "
      << pose[1] << ", "
      << pose[2] << ", "
      << pose[3] << ", "
      << pose[4] << ", "
      << pose[5] << ", "
      << pose[6] << ", "
      << pose[7] << ", "
      << pose[8] << std::endl;

    position_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    velocity_ = Eigen::Vector3d(pose[3], pose[4], pose[5]);
    Eigen::Vector3d vAi(pose[6], pose[7], pose[8]);
    Eigen::AngleAxisd vAAi(vAi.norm(), vAi.normalized());
    rotation_ = Eigen::Quaterniond(vAAi);
    bias_ = Eigen::Vector3d(tracker_.gyr_bias.x,
      tracker_.gyr_bias.y,
      tracker_.gyr_bias.z);
    // Change this
    valid_ = Valid();
    time_ = msg->header.stamp;
  }

  if (valid_) {
    Update(*msg);
    valid_ = Valid();
  }

  lastmsgwasimu_ = false;

  return;
}

bool ViveEKF::Valid() {
  // check_validity
  // HERE
  return true;
}

bool ViveEKF::GetTransform(geometry_msgs::TransformStamped& msg) {
  if (!valid_) return false;

  // Convert imu to ligh frame
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi(tQi);
  // New name for better readability
  Eigen::Vector3d vPi = position_;
  Eigen::Quaterniond vQi = rotation_;
  Eigen::Matrix3d vRi(vQi);
  // Transform to light frame
  Eigen::Vector3d vPt = vRi * (-tRi.transpose() * tPi) + vPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();
  Eigen::Quaterniond vQt(vRt);

  // Change to this to be in the light frame
  msg.transform.translation.x = vRt(0);
  msg.transform.translation.y = vRt(1);
  msg.transform.translation.z = vRt(2);
  msg.transform.rotation.w = vQt.w();
  msg.transform.rotation.x = vQt.x();
  msg.transform.rotation.y = vQt.y();
  msg.transform.rotation.z = vQt.z();
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

    H.block<1,13>(row,0) = HorizontalMeasureModelDiff(vPt_x, vPt_y, vPt_z,
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
    H.block<1,13>(row,0) = VerticalMeasureModelDiff(vPt_x, vPt_y, vPt_z,
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
  Eigen::MatrixXd slicedR(sensors.size(), sensors.size());
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
  // Time difference
  double dT = (msg.header.stamp - time_).toSec();
  if (!lastmsgwasimu_) dT = 2*dT;

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

  // Search for outliers
  hive::ViveLight clean_msg = msg;
  auto msg_it = clean_msg.samples.begin();
  while (msg_it != clean_msg.samples.end()) {
    if (msg_it->angle > M_PI / 3 || msg_it->angle < -M_PI / 3)
      clean_msg.samples.erase(msg_it);
    else
      msg_it++;
  }

  size_t row = 0;
  Eigen::VectorXd Z(clean_msg.samples.size());
  std::vector<int> sensors;
  for (auto sample : clean_msg.samples) {
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

  int total_sensors = tracker_.sensors.size();
  // Choose the model according to the orientation
  if (clean_msg.axis == HORIZONTAL) {
    // Derivative of measurement model
    H = GetHorizontalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Difference between prediction and real measures
    Y = Z - GetHorizontalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Sliced measurement covariance matrix
    R = SliceR(measure_covariance_.block(total_sensors * HORIZONTAL,
      total_sensors * HORIZONTAL, total_sensors, total_sensors), sensors);
    // std::cout << "HORIZONTAL" << std::endl;
    // std::cout << "Z: " << Z.transpose() << std::endl;
    // std::cout << "Y: " << Y.transpose() << std::endl;
  } else {
    // Derivative of measurement model
    H = GetVerticalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // std::cout << "H:\n" << H << std::endl;
    // Difference between prediction and real measures
    // std::cout << "Z:\n" << Z << std::endl;
    // std::cout << "estZ:\n" << GetVerticalZ(position_, rotation_, sensors,
      // tracker_, environment_.lighthouses[clean_msg.lighthouse],
      // lighthouses_[clean_msg.lighthouse], correction_) << std::endl;
    Y = Z - GetVerticalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // std::cout << "Y:\n" << Y << std::endl;
    // Sliced measurement covariance matrix
    R = SliceR(measure_covariance_.block(total_sensors * VERTICAL,
      total_sensors  *VERTICAL, total_sensors, total_sensors), sensors);
    // std::cout << "R:\n" << R << std::endl;
    // std::cout << "VERTICAL" << std::endl;
    // std::cout << "Z: " << Z.transpose() << std::endl;
    // std::cout << "Y: " << Y.transpose() << std::endl;
  }

  // Kalman Gain
  K = oldP * H.transpose() * (H * oldP * H.transpose() + R).inverse();
  // std::cout << "K:\n" << K << std::endl;
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
  time_ = clean_msg.header.stamp;

  return true;
}

void ViveEKF::PrintState() {
  // std::cout << "Position: "
  //   << position_(0) << ", "
  //   << position_(1) << ", "
  //   << position_(2) << std::endl;
  // std::cout << "Velocity: "
  //   << velocity_(0) << ", "
  //   << velocity_(1) << ", "
  //   << velocity_(2) << std::endl;
  // std::cout << "Orientation: "
  //   << rotation_.w() << ", "
  //   << rotation_.x() << ", "
  //   << rotation_.y() << ", "
  //   << rotation_.z() << std::endl;
  // std::cout << "Bias: "
  //   << bias_(0) << ", "
  //   << bias_(1) << ", "
  //   << bias_(2) << std::endl;
  // std::cout << "Covariance: "
  //   << covariance_.block<3,3>(0,0).trace() << ", "
  //   << covariance_.block<3,3>(3,3).trace() << ", "
  //   << covariance_.block<4,4>(6,6).trace() << ", "
  //   << covariance_.block<3,3>(10,10).trace() << std::endl;
  std::cout << position_(0) << ", "
    << position_(1) << ", "
    << position_(2) << ", ";
  Eigen::AngleAxisd AA(rotation_);
  std::cout << AA.axis()(0) * AA.angle() << ", "
    << AA.axis()(1) * AA.angle() << ", "
    << AA.axis()(2) * AA.angle() << std::endl;
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
      ekf_map[tr.serial] = ViveEKF(
        cal.trackers[tr.serial],
        cal.lighthouses,
        cal.environment,
        1.0, 1e-1, false);
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
      std::cout << "LIG  ";// << (int)vl->axis << std::endl;
      ekf_map[vl->header.frame_id].ProcessLight(vl);
      ekf_map[vl->header.frame_id].PrintState();
      counter++;
    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      std::cout << "IMU  ";// << std::endl;
      ekf_map[vi->header.frame_id].ProcessImu(vi);
      ekf_map[vi->header.frame_id].PrintState();
      counter++;
    }
    // std::cout << std::endl;
    // if (counter >= 730) break;
  }
  ROS_INFO("Data processment complete.");

  return 0;

}