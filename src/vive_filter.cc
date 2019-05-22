// ROS includes
#include <hive/vive_filter.h>
#include <hive/vive_filter_diff.h>

namespace filter {
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

  class ViveHorizontalCost
  {
  public:
    // Constructor
    ViveHorizontalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl, // vive to lighthouse
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveHorizontalCost();
    // Ceres operator
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
    geometry_msgs::Transform vTl_;
  };

  // Light cost - Cost using the poses from the imu frame to the vive frame
  class ViveVerticalCost
  {
  public:
    // Constructor
    ViveVerticalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl, // Vive to lighthouse
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveVerticalCost();
    // Ceres operator
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
    geometry_msgs::Transform vTl_;
  };

  ViveHorizontalCost::ViveHorizontalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl,
      Tracker tracker,
      Motor lighthouse,
      bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    vTl_ = vTl;
    return;
  }

  ViveHorizontalCost::~ViveHorizontalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveHorizontalCost::operator()(const T* const * parameters,
    T* residual) const {
    // Optimization parameters
    Eigen::Matrix<T, 3, 1> vPi;
    vPi << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRi;
    ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

    // Lighthouse pose
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << T(vTl_.translation.x),
      T(vTl_.translation.y),
      T(vTl_.translation.z);
    Eigen::Quaternion<T> vQl(
      T(vTl_.rotation.w),
      T(vTl_.rotation.x),
      T(vTl_.rotation.y),
      T(vTl_.rotation.z));
    Eigen::Matrix<T, 3, 3> vRl;
    vRl = vQl.toRotationMatrix();

    // Inertial transform
    Eigen::Matrix<T,3,1> tPi;
    tPi << T(tracker_.imu_transform.translation.x),
      T(tracker_.imu_transform.translation.y),
      T(tracker_.imu_transform.translation.z);
    Eigen::Quaternion<T> tQi(
      T(tracker_.imu_transform.rotation.w),
      T(tracker_.imu_transform.rotation.x),
      T(tracker_.imu_transform.rotation.y),
      T(tracker_.imu_transform.rotation.z));
    Eigen::Matrix<T, 3, 3> tRi;
    tRi = tQi.toRotationMatrix();

    // Invert inertial transform
    Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
    Eigen::Matrix<T,3,3> iRt = tRi.transpose();

    // Vive and IMU frame to lighthouse and tracker
    Eigen::Matrix<T, 3, 1> lPv;
    lPv = - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRv;
    lRv = vRl.transpose();
    Eigen::Matrix<T, 3, 1> lPt;
    lPt = lRv * (vRi * iPt + vPi) + lPv;
    Eigen::Matrix<T, 3, 3> lRt;
    lRt = lRv * vRi * iRt;

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }

      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }

  ViveVerticalCost::ViveVerticalCost(hive::ViveLight data,
    geometry_msgs::Transform vTl,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    vTl_ = vTl;
    return;
  }

  ViveVerticalCost::~ViveVerticalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveVerticalCost::operator()(const T* const * parameters,
    T* residual) const {
    // Optimization parameters
    Eigen::Matrix<T, 3, 1> vPi;
    vPi << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRi;
    ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

    // Lighthouse pose
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << T(vTl_.translation.x),
      T(vTl_.translation.y),
      T(vTl_.translation.z);
    Eigen::Quaternion<T> vQl(
      T(vTl_.rotation.w),
      T(vTl_.rotation.x),
      T(vTl_.rotation.y),
      T(vTl_.rotation.z));
    Eigen::Matrix<T, 3, 3> vRl;
    vRl = vQl.toRotationMatrix();

    // Inertial transform
    Eigen::Matrix<T,3,1> tPi;
    tPi << T(tracker_.imu_transform.translation.x),
      T(tracker_.imu_transform.translation.y),
      T(tracker_.imu_transform.translation.z);
    Eigen::Quaternion<T> tQi(
      T(tracker_.imu_transform.rotation.w),
      T(tracker_.imu_transform.rotation.x),
      T(tracker_.imu_transform.rotation.y),
      T(tracker_.imu_transform.rotation.z));
    Eigen::Matrix<T, 3, 3> tRi;
    tRi = tQi.toRotationMatrix();

    // Invert inertial transform
    Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
    Eigen::Matrix<T,3,3> iRt = tRi.transpose();

    // Vive and IMU frame to lighthouse and tracker
    Eigen::Matrix<T, 3, 1> lPv;
    lPv = - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRv;
    lRv = vRl.transpose();
    Eigen::Matrix<T, 3, 1> lPt;
    lPt = lRv * (vRi * iPt + vPi) + lPv;
    Eigen::Matrix<T, 3, 3> lRt;
    lRt = lRv * vRi * iRt;

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }


      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }

}

ViveFilter::ViveFilter() {}

ViveFilter::ViveFilter(Tracker & tracker,
  std::map<std::string, Lighthouse> & lighthouses,
  Environment & environment,
  double model_noise, // time update
  double measure_noise, // measurements
  bool correction,
  filter::type ftype) {
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
  covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  filter_type_ = ftype;
  ext_covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE + NOISE_SIZE,
    STATE_SIZE + NOISE_SIZE);
  ext_covariance_.block<STATE_SIZE, STATE_SIZE>(0,0) = covariance_;
  ext_covariance_.block<NOISE_SIZE, NOISE_SIZE>(
    STATE_SIZE, STATE_SIZE) = model_covariance_;
  return;
}

ViveFilter::ViveFilter(Tracker & tracker,
  std::map<std::string, Lighthouse> & lighthouses,
  Environment & environment,
  Eigen::MatrixXd model_noise, // time update
  Eigen::MatrixXd measure_noise, // measurements
  bool correction,
  filter::type ftype) {
  position_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  rotation_ = Eigen::Quaterniond::Identity();
  velocity_ = Eigen::Vector3d::Zero();
  bias_ = Eigen::Vector3d::Zero();
  tracker_ = tracker;
  lighthouses_ = lighthouses;
  environment_ = environment;
  valid_ = false;
  correction_ = correction;
  if (model_noise.cols() != NOISE_SIZE ||
    model_noise.rows() != NOISE_SIZE) throw;
  model_covariance_ = model_noise;
  // measure_covariance_ - double the size for both sweeps. The
  // first half of the matrix is for the horizontal, the second half
  // of the matrix is for the vertical sweep
  //    [ H  HV ]
  //    [ VH  V ]
  if (measure_noise.cols() != 2*tracker_.sensors.size() ||
    measure_noise.rows() != 2*tracker_.sensors.size()) throw;
  measure_covariance_ = measure_noise;
  covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  // UKF's extended covariance.
  ext_covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE + NOISE_SIZE,
    STATE_SIZE + NOISE_SIZE);
  ext_covariance_.block<STATE_SIZE, STATE_SIZE>(0,0) = covariance_;
  ext_covariance_.block<NOISE_SIZE, NOISE_SIZE>(
    STATE_SIZE, STATE_SIZE) = model_covariance_;
  filter_type_ = ftype;
  return;
}

ViveFilter::~ViveFilter() {
  return;
}

bool ViveFilter::Initialize() {
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
      ceres::DynamicAutoDiffCostFunction<filter::ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<filter::ViveHorizontalCost, 4>
        (new filter::ViveHorizontalCost(sample,
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
      ceres::DynamicAutoDiffCostFunction<filter::ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<filter::ViveVerticalCost, 4>
        (new filter::ViveVerticalCost(sample,
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
  options.max_solver_time_in_seconds = 0.5;
  ceres::Solve(options, &problem, &summary);

  if (summary.final_cost > 1e-4) return false;

  std::cout << "init ";
  ROS_FATAL("INIT");

  position_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  velocity_ = Eigen::Vector3d(pose[3], pose[4], pose[5]);
  Eigen::Vector3d vAi(pose[6], pose[7], pose[8]);
  Eigen::AngleAxisd vAAi(vAi.norm(), vAi.normalized());
  rotation_ = Eigen::Quaterniond(vAAi);
  bias_ = Eigen::Vector3d(tracker_.gyr_bias.x,
    tracker_.gyr_bias.y,
    tracker_.gyr_bias.z);
  covariance_ = HIVE_APE_ACC * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  // UKF's extended covariance.
  ext_covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE + NOISE_SIZE,
    STATE_SIZE + NOISE_SIZE);
  ext_covariance_.block<STATE_SIZE, STATE_SIZE>(0,0) = covariance_;
  ext_covariance_.block<NOISE_SIZE, NOISE_SIZE>(
    STATE_SIZE, STATE_SIZE) = model_covariance_;
  // Change this
  time_ = light_data_.back().header.stamp;
  return true;
}

void ViveFilter::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) return;
  if (!valid_) return;

  switch(filter_type_) {
    case filter::ekf:
      PredictEKF(*msg);
      break;
    case filter::iekf:
      PredictIEKF(*msg);
      break;
    case filter::ukf:
      PredictUKF(*msg);
      break;
    default:
      std::cout << "Method not available\n";
  }

  valid_ = Valid();
  used_ = false;
  lastmsgwasimu_ = true;

  return;
}

void ViveFilter::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return;

  light_data_.push_back(*msg);
  while (light_data_.size() > LIGHT_DATA_BUFFER) {
    light_data_.erase(light_data_.begin());
  }

  // Solve rapidly
  if (!valid_ && light_data_.size() >= LIGHT_DATA_BUFFER) {
    Initialize();
  }

  // Update estimate
  if (valid_) {
    switch (filter_type_) {
      case filter::ekf:
        UpdateEKF(*msg);
        break;
      case filter::iekf:
        UpdateIEKF(*msg);
        break;
      case filter::ukf:
        UpdateUKF(*msg);
        break;
      default:
        std::cout << "Method not available\n";
    }
  }

  valid_ = Valid();
  used_ = false;
  lastmsgwasimu_ = false;

  return;
}

bool ViveFilter::GetTransform(geometry_msgs::TransformStamped& msg) {
  if (!valid_ || used_) return false;

  // Convert imu to ligh frame
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();
  // New name for better readability
  Eigen::Vector3d vPi = position_;
  Eigen::Quaterniond vQi = rotation_;
  Eigen::Matrix3d vRi = vQi.toRotationMatrix();
  // Transform to light frame
  Eigen::Vector3d vPt = vRi * (-tRi.transpose() * tPi) + vPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();
  Eigen::Quaterniond vQt(vRt);

  // Change to this to be in the light frame
  msg.transform.translation.x = vPt(0);
  msg.transform.translation.y = vPt(1);
  msg.transform.translation.z = vPt(2);
  msg.transform.rotation.w = vQt.w();
  msg.transform.rotation.x = vQt.x();
  msg.transform.rotation.y = vQt.y();
  msg.transform.rotation.z = vQt.z();
  msg.child_frame_id = tracker_.serial;
  msg.header.stamp = time_;
  msg.header.frame_id = "vive";

  used_ = true;

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
  F.block<4,3>(6,10) = -0.5 * filter::GetOmega(rotation);
  F.block<3,3>(10,0) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,3>(10,3) = Eigen::MatrixXd::Zero(3,3);
  F.block<3,4>(10,6) = Eigen::MatrixXd::Zero(3,4);
  F.block<3,3>(10,10) = Eigen::MatrixXd::Zero(3,3);

  // TODO check this over here
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
  F(4,9) = tA_x*vQt_w*-2.0+tA_y*vQt_z*4.0-tA_z*vQt_y*2.0;
  F(5,9) = tA_x*vQt_x*-2.0-tA_y*vQt_y*2.0;

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
  Eigen::Vector3d light_P_imu = filter::ConvertMessage(
    tracker.imu_transform.translation);
  Eigen::Quaterniond light_Q_imu = filter::ConvertMessage(
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
  Eigen::Matrix3d vRl = filter::ConvertMessage(
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
  Eigen::Vector3d light_P_imu = filter::ConvertMessage(
    tracker.imu_transform.translation);
  Eigen::Quaterniond light_Q_imu = filter::ConvertMessage(
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
  Eigen::Matrix3d vRl = filter::ConvertMessage(
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
  G.block<4,3>(6,0) = filter::GetOmega(rotation);
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
  Eigen::Vector3d acc_bias,
  Eigen::Vector3d ang_bias) {
  Eigen::VectorXd dotX(STATE_SIZE);
  // Position
  dotX.segment<3>(0) = velocity;
  // Velocity - assumed constant
  dotX.segment<3>(3) = - rotation.toRotationMatrix() * (linear_acceleration - acc_bias) + gravity;
  // Orientation
  Eigen::MatrixXd Omega = filter::GetOmega(rotation);
  dotX.segment<4>(6) = 0.5 * (Omega * (angular_velocity - ang_bias));
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
  Eigen::Vector3d vPl = filter::ConvertMessage(lhTransform.translation);
  Eigen::Matrix3d vRl = filter::ConvertMessage(lhTransform.rotation).toRotationMatrix();
  Eigen::Vector3d lPv = - vRl.transpose() * vPl;
  Eigen::Matrix3d lRv = vRl.transpose();
  // lightPimu - transform the imu frame to the light frame
  Eigen::Vector3d tlPt = filter::ConvertMessage(
    tracker.imu_transform.translation);
  // lightRimu - transform the imu frame to the light frame
  Eigen::Matrix3d tlRt = filter::ConvertMessage(
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
    Eigen::Vector3d tlPs = filter::ConvertMessage(tracker.sensors[sensor].position);
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
  Eigen::Vector3d vPi = position;
  Eigen::Matrix3d vRi = rotation.toRotationMatrix();
  Eigen::Vector3d vPl = filter::ConvertMessage(lhTransform.translation);
  Eigen::Matrix3d vRl = filter::ConvertMessage(lhTransform.rotation).toRotationMatrix();
  Eigen::Vector3d lPv = - vRl.transpose() * vPl;
  Eigen::Matrix3d lRv = vRl.transpose();
  // lightPimu - transform the imu frame to the light frame
  Eigen::Vector3d tPi = filter::ConvertMessage(
    tracker.imu_transform.translation);
  // lightRimu - transform the imu frame to the light frame
  Eigen::Matrix3d tRi = filter::ConvertMessage(
    tracker.imu_transform.rotation).toRotationMatrix();
  // conversion from tlTt to tTtl
  Eigen::Vector3d tPt = - tRi.transpose() * tPi;
  Eigen::Matrix3d tRt = tRi.transpose();
  // Lighthouse parameters
  double phase = lhSpecs.vertical_motor.phase;
  double tilt = lhSpecs.vertical_motor.tilt;
  double gib_phase = lhSpecs.vertical_motor.gib_phase;
  double gib_mag = lhSpecs.vertical_motor.gib_magnitude;
  double curve = lhSpecs.vertical_motor.curve;

  size_t row = 0;
  for (auto sensor : sensors) {
    Eigen::Vector3d tPs = filter::ConvertMessage(tracker.sensors[sensor].position);
    Eigen::Vector3d lPs = lRv * ( vRi * (tRt * tPs + tPt) + vPi ) + lPv;

    // std::cout << "lPs " << sensor << ": " << lPs(0) << ", " << lPs(1) << ", " << lPs(2) << std::endl;

    double x = (lPs(0)/lPs(2));
    double y = (lPs(1)/lPs(2));

    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;

    // std::cout << "phase: " << phase << std::endl;
    // std::cout << "tilt: " << tilt << std::endl;
    // std::cout << "curve: " << curve << std::endl;
    // std::cout << "gib_phase: " << gib_phase << std::endl;
    // std::cout << "gib_mag: " << gib_mag << std::endl;
    if (correction) {
      Z(row) = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      // std::cout << "Z: " << Z(row) << std::endl;
    } else {
      Z(row) = atan(y);
    }
    row++;
  }
  return Z;
}

// Check_validity
bool ViveFilter::Valid() {
  if (std::isnan(position_(0)) ||
    std::isnan(position_(1)) ||
    std::isnan(position_(2)) ||
    std::isnan(velocity_(0)) ||
    std::isnan(velocity_(1)) ||
    std::isnan(velocity_(2)) ||
    std::isnan(rotation_.w()) ||
    std::isnan(rotation_.x()) ||
    std::isnan(rotation_.y()) ||
    std::isnan(rotation_.z()) ||
    std::isnan(bias_(0)) ||
    std::isnan(bias_(1)) ||
    std::isnan(bias_(2))) return false;

  if (covariance_.trace() == 0) return false;

  double cost = 0;
  double light_counter = 0;
  for (auto light_msg : light_data_) {
    // Pose of the imu in the vive frame
    Eigen::Vector3d vPi = position_;
    Eigen::Matrix3d vRi = rotation_.toRotationMatrix();
    // Pose of the imu in the light frame
    Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
      tracker_.imu_transform.translation.y,
      tracker_.imu_transform.translation.z);
    Eigen::Matrix3d tRi = Eigen::Quaterniond(tracker_.imu_transform.rotation.w,
      tracker_.imu_transform.rotation.x,
      tracker_.imu_transform.rotation.y,
      tracker_.imu_transform.rotation.z).toRotationMatrix();
    // Pose of the lighthouse in the vive frame
    Eigen::Vector3d vPl(
      environment_.lighthouses[light_msg.lighthouse].translation.x,
      environment_.lighthouses[light_msg.lighthouse].translation.y,
      environment_.lighthouses[light_msg.lighthouse].translation.z);
    Eigen::Matrix3d vRl = Eigen::Quaterniond(
      environment_.lighthouses[light_msg.lighthouse].rotation.w,
      environment_.lighthouses[light_msg.lighthouse].rotation.x,
      environment_.lighthouses[light_msg.lighthouse].rotation.y,
      environment_.lighthouses[light_msg.lighthouse].rotation.z).toRotationMatrix();
    // Pose of the light frame in the lighthouse frame
    Eigen::Vector3d lPt = vRl.transpose() * (
      vRi * ( - tRi.transpose() * tPi) + vPi) + (- vRl.transpose() * vPl);
    Eigen::Matrix3d lRt = vRl.transpose() * vRi * tRi.transpose();

    // Mahalanobis distance
    // USed sensors
    std::vector<int> sensors;
    // Measured Angle
    std::vector<double> msAngle;
    // Predicted Angle
    std::vector<double> prAngle;

    for (auto sample : light_msg.samples) {
      // Check for outliers
      if (sample.angle > M_PI / 3 || sample.angle < - M_PI / 3) continue;
      // Sensor in the light frame
      Eigen::Vector3d tPs(tracker_.sensors[sample.sensor].position.x,
        tracker_.sensors[sample.sensor].position.y,
        tracker_.sensors[sample.sensor].position.z);
      // Sensor in lighthouse frame
      Eigen::Vector3d lPs = lRt * tPs + lPt;
      // Horizontal Angle
      if (light_msg.axis == HORIZONTAL) {
        // Angles
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_msg.lighthouse].horizontal_motor.phase;
        double tilt = lighthouses_[light_msg.lighthouse].horizontal_motor.tilt;
        double gib_phase = lighthouses_[light_msg.lighthouse].horizontal_motor.gib_phase;
        double gib_mag = lighthouses_[light_msg.lighthouse].horizontal_motor.gib_magnitude;
        double curve = lighthouses_[light_msg.lighthouse].horizontal_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
        } else {
          ang = atan(x);
        }
        // Adding to cost
        msAngle.push_back(sample.angle);
        prAngle.push_back(ang);
        sensors.push_back(sample.sensor);
      // Vertical Angle
      } else if (light_msg.axis == VERTICAL) {
        // Angles
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_msg.lighthouse].vertical_motor.phase;
        double tilt = lighthouses_[light_msg.lighthouse].vertical_motor.tilt;
        double gib_phase = lighthouses_[light_msg.lighthouse].vertical_motor.gib_phase;
        double gib_mag = lighthouses_[light_msg.lighthouse].vertical_motor.gib_magnitude;
        double curve = lighthouses_[light_msg.lighthouse].vertical_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
        } else {
          ang = atan(y);
        }
        // Adding to cost
        msAngle.push_back(sample.angle);
        prAngle.push_back(ang);
        sensors.push_back(sample.sensor);
      }
    }
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::VectorXd msEigenAngle(msAngle.size());
    Eigen::VectorXd prEigenAngle(prAngle.size());
    for (size_t i = 0; i < msAngle.size(); i++)
      msEigenAngle(i) = msAngle[i];
    for (size_t i = 0; i < prAngle.size(); i++)
      prEigenAngle(i) = prAngle[i];
    int total_sensors = tracker_.sensors.size();
    if (light_msg.axis == HORIZONTAL) {
      // Derivative of measurement model
      H = GetHorizontalH(position_, rotation_, sensors,
        tracker_, environment_.lighthouses[light_msg.lighthouse],
        lighthouses_[light_msg.lighthouse], correction_);
      // Sliced measurement covariance matrix
      R = filter::SliceR(measure_covariance_.block(total_sensors * HORIZONTAL,
        total_sensors * HORIZONTAL, total_sensors, total_sensors), sensors);
    } else {
      // Derivative of measurement model
      H = GetVerticalH(position_, rotation_, sensors,
        tracker_, environment_.lighthouses[light_msg.lighthouse],
        lighthouses_[light_msg.lighthouse], correction_);
      // Sliced measurement covariance matrix
      R = filter::SliceR(measure_covariance_.block(total_sensors * VERTICAL,
        total_sensors * VERTICAL, total_sensors, total_sensors), sensors);
    }
    // Mahalanobis cost right here
    Eigen::MatrixXd V = R + H * covariance_ * H.transpose();
    // std::cout << "R: " << R << std::endl;
    // std::cout << "Cov: " << covariance_ << std::endl;
    // std::cout << "H: " << H << std::endl;
    // std::cout << "V: " << V << std::endl;
    // Temporary
    // V = Eigen::MatrixXd::Identity(V.rows(), V.cols());
    cost += (msEigenAngle - prEigenAngle).transpose() * V.inverse() * (msEigenAngle - prEigenAngle);
    // cost += (msEigenAngle - prEigenAngle).transpose() * (msEigenAngle - prEigenAngle);
    // std::cout << "singleCost: " << 
    //   (msEigenAngle - prEigenAngle).transpose() * V.inverse() * (msEigenAngle - prEigenAngle) << std::endl;
    light_counter++;
    // std::cout << "V^-1: " << V.inverse() << std::endl;
    // std::cout << "changedCost: " << (msEigenAngle - prEigenAngle).transpose() * (msEigenAngle - prEigenAngle) << std::endl;
    // std::cout << "thisCost: " << (msEigenAngle - prEigenAngle).transpose() * V.inverse() * (msEigenAngle - prEigenAngle) << std::endl;
  }

  if (cost > pow(MAHALANOBIS_MAX_DIST,2) * light_counter) {
  // if (cost > 1e-2 * light_counter) {
    // std::cout << "Not valid ";
    // TODO Change this
    return false;
  }

  // std::cout << "Valid ";
  return true;
}

// Time update (Inertial data)
bool ViveFilter::PredictIEKF(const sensor_msgs::Imu & msg) {
  // It's the same method
  return PredictEKF(msg);
}

// Time update (Inertial data)
bool ViveFilter::PredictEKF(const sensor_msgs::Imu & msg) {
  // std::cout << "Predict" << std::endl;
  // Convert measurements
  Eigen::Vector3d linear_acceleration = filter::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d angular_velocity = filter::ConvertMessage(msg.angular_velocity);
  Eigen::Vector3d gravity = filter::ConvertMessage(environment_.gravity);
  // Time difference
  double dT = (msg.header.stamp - time_).toSec();
  if (!lastmsgwasimu_) dT = 2*dT;

  Eigen::Vector3d acc_bias(tracker_.acc_bias.x,
    tracker_.acc_bias.y,
    tracker_.acc_bias.z);
  // Derivative of the state in time
  Eigen::VectorXd dState = GetDState(velocity_,
    rotation_,
    linear_acceleration,
    angular_velocity,
    gravity,
    acc_bias,
    bias_);

  // std::cout << "cG: " << gravity.transpose() << std::endl;
  // std::cout << "mG: " << (rotation_.toRotationMatrix() * linear_acceleration).transpose() << std::endl;
  // std::cout << "dState: " << dState.transpose() << std::endl;
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
  Eigen::MatrixXd Pnoise = Eigen::MatrixXd::Zero(STATE_SIZE,
    STATE_SIZE);
  // Pnoise(0,0) = 1e-2;
  // Pnoise(1,1) = 1e-2;
  // Pnoise(2,2) = 1e-2;
  // Pnoise(3,3) = 1e-2;
  // Pnoise(4,4) = 1e-2;
  // Pnoise(5,5) = 1e-2;
  Eigen::MatrixXd newP = (I + dT * F) *
    oldP * (I + dT * F).transpose() + (dT * dT) * (G * Q * G.transpose()) + Pnoise;

  // New state
  Eigen::VectorXd newX = oldX + dT * dState;
  // std::cout << "oldX: " << oldX.transpose() << std::endl;
  // std::cout << "dT * dState: " << dT * dState.transpose() << std::endl;
  // std::cout << "newX: " << newX.transpose() << std::endl;

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
bool ViveFilter::UpdateEKF(const hive::ViveLight & msg) {
  // std::cout << "Update" << std::endl;
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
  Eigen::VectorXd pZ;
  std::vector<int> sensors;
  for (auto sample : clean_msg.samples) {
    // Put angle in Vector
    Z(row, 0) = sample.angle;
    row++;
    // For later usage
    sensors.push_back(sample.sensor);
    // std::cout << sample.sensor << " - "
    //   << tracker_.sensors[sample.sensor].position.x << " "
    //   << tracker_.sensors[sample.sensor].position.y << " "
    //   << tracker_.sensors[sample.sensor].position.z << std::endl;
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
    // std::cout << "HORIZONTAL" << std::endl;
    // std::cout << environment_.lighthouses[clean_msg.lighthouse].translation.x << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].translation.y << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].translation.z << " | "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.w << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.x << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.y << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.z << std::endl;
    // std::cout << tracker_.imu_transform.translation.x << ", "
    //   << tracker_.imu_transform.translation.y << ", "
    //   << tracker_.imu_transform.translation.z << " | "
    //   << tracker_.imu_transform.rotation.w << ", "
    //   << tracker_.imu_transform.rotation.x << ", "
    //   << tracker_.imu_transform.rotation.y << ", "
    //   << tracker_.imu_transform.rotation.z << std::endl;
    // std::cout << "Phase: " << lighthouses_[clean_msg.lighthouse].horizontal_motor.phase << "\n"
    //   << "Tilt: " << lighthouses_[clean_msg.lighthouse].horizontal_motor.tilt << "\n"
    //   << "gib_phase: " << lighthouses_[clean_msg.lighthouse].horizontal_motor.gib_phase << "\n"
    //   << "gib_magnitude: " << lighthouses_[clean_msg.lighthouse].horizontal_motor.gib_magnitude << "\n"
    //   << "curve: " << lighthouses_[clean_msg.lighthouse].horizontal_motor.curve << "\n";
    // Derivative of measurement model
    H = GetHorizontalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Difference between prediction and real measures
    pZ = GetHorizontalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    Y = Z - pZ;
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * HORIZONTAL,
      total_sensors * HORIZONTAL, total_sensors, total_sensors), sensors);
  } else {
    // std::cout << "VERTICAL" << std::endl;
    // std::cout << environment_.lighthouses[clean_msg.lighthouse].translation.x << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].translation.y << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].translation.z << " | "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.w << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.x << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.y << ", "
    //   << environment_.lighthouses[clean_msg.lighthouse].rotation.z << std::endl;
    // std::cout << tracker_.imu_transform.translation.x << ", "
    //   << tracker_.imu_transform.translation.y << ", "
    //   << tracker_.imu_transform.translation.z << " | "
    //   << tracker_.imu_transform.rotation.w << ", "
    //   << tracker_.imu_transform.rotation.x << ", "
    //   << tracker_.imu_transform.rotation.y << ", "
    //   << tracker_.imu_transform.rotation.z << std::endl;
    // std::cout << "Phase: " << lighthouses_[clean_msg.lighthouse].vertical_motor.phase << "\n"
    //   << "Tilt: " << lighthouses_[clean_msg.lighthouse].vertical_motor.tilt << "\n"
    //   << "gib_phase: " << lighthouses_[clean_msg.lighthouse].vertical_motor.gib_phase << "\n"
    //   << "gib_magnitude: " << lighthouses_[clean_msg.lighthouse].vertical_motor.gib_magnitude << "\n"
    //   << "curve: " << lighthouses_[clean_msg.lighthouse].vertical_motor.curve << "\n";
    // Derivative of measurement model
    H = GetVerticalH(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Difference between prediction and real measures
    pZ = GetVerticalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    Y = Z - pZ;
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * VERTICAL,
      total_sensors  *VERTICAL, total_sensors, total_sensors), sensors);
  }

  // Kalman Gain
  // Eigen::MatrixXd AUX = 1e-6 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  // K = AUX * H.transpose() * (H * AUX * H.transpose() + R).inverse();
  // R = Eigen::MatrixXd::Identity(R.rows(), R.cols());
  K = oldP * H.transpose() * (H * oldP * H.transpose() + R).inverse();
  newX = oldX + K * Y;
  // std::cout << "oldP:\n" << oldP << std::endl;
  // std::cout << "H:\n" << H << std::endl;
  // std::cout << "Z: " << (Z).transpose() << std::endl;
  // std::cout << "pZ: " << (pZ).transpose() << std::endl;
  // std::cout << "Y: " << (Y).transpose() << std::endl;
  // std::cout << "K*Y: " << (K*Y).transpose() << std::endl;
  // std::cout << "oldX: " << oldX.transpose() << std::endl;
  // std::cout << "newX: " << newX.transpose() << std::endl;
  newP = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * oldP;
  // std::cout << "newP " << newP << std::endl;

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

bool ViveFilter::UpdateIEKF(const hive::ViveLight & msg) {
  int total_sensors = tracker_.sensors.size();

  // Search for outliers
  hive::ViveLight clean_msg = msg;
  auto sample_it = clean_msg.samples.begin();
  while (sample_it != clean_msg.samples.end()) {
    if (sample_it->angle > M_PI / 3 || sample_it->angle < -M_PI / 3)
      clean_msg.samples.erase(sample_it);
    else
      sample_it++;
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
  Eigen::VectorXd next_tmpX = oldX;
  Eigen::VectorXd prev_tmpX = oldX;
  oldP = Eigen::MatrixXd(covariance_);

  // Choose the model according to the orientation
  if (clean_msg.axis == HORIZONTAL) {
    // Difference between prediction and real measures
    Y = Z - GetHorizontalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * HORIZONTAL,
      total_sensors * HORIZONTAL, total_sensors, total_sensors), sensors);
  } else {
    // Difference between prediction and real measures
    Y = Z - GetVerticalZ(position_, rotation_, sensors,
      tracker_, environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_);
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * VERTICAL,
      total_sensors  *VERTICAL, total_sensors, total_sensors), sensors);
  }

  double error = 9e9;
  size_t counter = 0;
  while(error > IEFK_THRESHOLD) {
    // Quickly convert position and rotation
    Eigen::Vector3d tmp_position(next_tmpX(0),
      next_tmpX(1),
      next_tmpX(2));
    Eigen::Quaterniond tmp_rotation(next_tmpX(6),
      next_tmpX(7),
      next_tmpX(8),
      next_tmpX(9));
    // Fix any carry on errors
    tmp_rotation.normalize();
    // Choose the model according to the orientation
    if (clean_msg.axis == HORIZONTAL) {
      // Derivative of measurement model
      H = GetHorizontalH(tmp_position, tmp_rotation, sensors,
        tracker_, environment_.lighthouses[clean_msg.lighthouse],
        lighthouses_[clean_msg.lighthouse], correction_);
    } else {
      // Derivative of measurement model
      H = GetVerticalH(tmp_position, tmp_rotation, sensors,
        tracker_, environment_.lighthouses[clean_msg.lighthouse],
        lighthouses_[clean_msg.lighthouse], correction_);
    }
    // Kalman Gain
    K = oldP * H.transpose() * (H * oldP * H.transpose() + R).inverse();
    // Update intermediate X
    next_tmpX = oldX + K * Y;
    // Error to check if it ends cycle
    error = (next_tmpX - prev_tmpX).norm();

    prev_tmpX = next_tmpX;
    counter++;
    if (counter >= 1000) break;
  }

  newX = oldX + K * Y;
  newP = Eigen::MatrixXd(
    (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * oldP);

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

// Vector that represents the evolution of the system is continuous time
Eigen::VectorXd GetExtendedDState(Eigen::Vector3d velocity,
  Eigen::Quaterniond rotation,
  Eigen::Vector3d linear_acceleration,
  Eigen::Vector3d angular_velocity,
  Eigen::Vector3d gravity,
  Eigen::Vector3d bias) {
  Eigen::VectorXd dotX(STATE_SIZE + NOISE_SIZE);
  // Position
  dotX.segment<3>(0) = velocity;
  // Velocity - assumed constant
  dotX.segment<3>(3) = - rotation.toRotationMatrix() * linear_acceleration + gravity;
  // Orientation
  Eigen::MatrixXd Omega = filter::GetOmega(rotation);
  dotX.segment<4>(6) = 0.5 * (Omega * (angular_velocity - bias));
  dotX.segment<3>(10) = Eigen::VectorXd::Zero(3);
  dotX.segment<NOISE_SIZE>(STATE_SIZE) =
    Eigen::VectorXd::Zero(NOISE_SIZE);
  // std::cout << "dotX: " << dotX.transpose() << std::endl;
  return dotX;
}

bool ViveFilter::PredictUKF(const sensor_msgs::Imu & msg) {
  // std::cout << "PredictUKF ";
  Eigen::VectorXd prev_extState(STATE_SIZE + NOISE_SIZE);
  Eigen::MatrixXd prev_extCovariance(STATE_SIZE + NOISE_SIZE,
    STATE_SIZE + NOISE_SIZE);

  // Convert measurements
  Eigen::Vector3d linear_acceleration = filter::ConvertMessage(msg.linear_acceleration);
  Eigen::Vector3d angular_velocity = filter::ConvertMessage(msg.angular_velocity);
  Eigen::Vector3d gravity = filter::ConvertMessage(environment_.gravity);

  // Time difference
  // double dT = 0.005;
  double dT = (msg.header.stamp - time_).toSec();
  if (!lastmsgwasimu_) dT = 2*dT;

  // Previous extended state estimate
  prev_extState.segment<3>(0) = position_;
  prev_extState.segment<3>(3) = velocity_;
  prev_extState(6) = rotation_.w();
  prev_extState(7) = rotation_.x();
  prev_extState(8) = rotation_.y();
  prev_extState(9) = rotation_.z();
  prev_extState.segment<3>(10) = bias_;
  prev_extState.segment<NOISE_SIZE>(13) =
    Eigen::VectorXd::Zero(NOISE_SIZE);

  // std::cout << "prev_extState: " << prev_extState.transpose() << std::endl;

  prev_extCovariance = ext_covariance_;

  std::vector<Eigen::VectorXd> prev_unscentedStates;
  std::vector<Eigen::VectorXd> next_unscentedStates;

  // std::cout << "prev_extCovariance: " << prev_extCovariance << std::endl;
  // std::cout << "squaredsigma: " << 
  //   ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
  //   prev_extCovariance << std::endl;

  // Eigen::MatrixXd sigma =
  //   (((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
  //   prev_extCovariance).sqrt();
  Eigen::MatrixXd sigma = Eigen::LLT<Eigen::MatrixXd>(
    ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    prev_extCovariance).matrixL();


  // std::cout << "sigma: " << sigma << std::endl;

  prev_unscentedStates.push_back(prev_extState);
  for (size_t i = 0; i < sigma.cols(); i++) {
    prev_unscentedStates.push_back(prev_extState + sigma.col(i));
    prev_unscentedStates.push_back(prev_extState - sigma.col(i));
    // std::cout << "prev_unscentedStates: " <<
    //   prev_unscentedStates.back().transpose() << std::endl;
  }

  // Next states
  Eigen::VectorXd next_extState = Eigen::VectorXd::Zero(STATE_SIZE + NOISE_SIZE);
  Eigen::MatrixXd next_extCovariance = 
    Eigen::MatrixXd::Zero(STATE_SIZE + NOISE_SIZE, STATE_SIZE + NOISE_SIZE);

  // std::cout << "HERE4" << std::endl;
  for (size_t i = 0; i < prev_unscentedStates.size(); i++) {
    // Format change
    Eigen::Vector3d velocity = prev_unscentedStates[i].segment<3>(3);
    Eigen::Quaterniond rotation (prev_unscentedStates[i](6),
      prev_unscentedStates[i](7),
      prev_unscentedStates[i](8),
      prev_unscentedStates[i](9));
    Eigen::Vector3d bias = prev_unscentedStates[i].segment<3>(10);
    // Get time derivative
    Eigen::VectorXd dState = GetExtendedDState(
      velocity, rotation, linear_acceleration, angular_velocity, gravity, bias);
    // Get next state and save it
    Eigen::VectorXd tmp_extState = prev_unscentedStates[i] + dT * dState;
    // std::cout << "prev_unscentedStates[i]: " << prev_unscentedStates[i].transpose() << std::endl;
    // std::cout << "dState: " << dState.transpose() << std::endl;
    // std::cout << "tmp_extState: " << tmp_extState.transpose() << std::endl;

    next_unscentedStates.push_back(tmp_extState);
  }


  next_extState = UKF_FACTOR * next_unscentedStates[0];
  for (size_t i = 1; i < next_unscentedStates.size(); i++) {
    next_extState += 0.5 * next_unscentedStates[i];
  }
  next_extState = 1.0 / ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    next_extState;

  // std::cout << "HERE5" << std::endl;
  // std::cout << "next_extState: " << next_extState.transpose() << std::endl;

  next_extCovariance += UKF_FACTOR * (next_unscentedStates[0] - next_extState) *
    (next_unscentedStates[0] - next_extState).transpose();
  for (size_t i = 1; i < next_unscentedStates.size(); i++) {
    next_extCovariance += 0.5 * (next_unscentedStates[i] - next_extState) *
    (next_unscentedStates[i] - next_extState).transpose();
  }
  next_extCovariance = 1.0 / ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    next_extCovariance;

  // std::cout << "next_extCovariance: " << next_extCovariance.transpose() << std::endl;
  // std::cout << "HERE6" << std::endl;
  // Save new state
  position_ = next_extState.segment<3>(0);
  velocity_ = next_extState.segment<3>(3);
  rotation_ = Eigen::Quaterniond(next_extState(6),
    next_extState(7),
    next_extState(8),
    next_extState(9)).normalized();
  bias_ = next_extState.segment<3>(10);
  covariance_ = next_extCovariance.block<STATE_SIZE, STATE_SIZE>(0,0);
  ext_covariance_ = next_extCovariance;
  time_ = msg.header.stamp;
  // std::cout << "HERE7" << std::endl;

  // exit(0);
  return true;
}

bool ViveFilter::UpdateUKF(const hive::ViveLight & msg) {
  // std::cout << "UpdateUKF ";
  Eigen::VectorXd prev_extState(STATE_SIZE + NOISE_SIZE);
  Eigen::MatrixXd prev_extCovariance(STATE_SIZE + NOISE_SIZE,
    STATE_SIZE + NOISE_SIZE);

  // Previous extended state estimate
  prev_extState.segment<3>(0) = position_;
  prev_extState.segment<3>(3) = velocity_;
  prev_extState(6) = rotation_.w();
  prev_extState(7) = rotation_.x();
  prev_extState(8) = rotation_.y();
  prev_extState(9) = rotation_.z();
  prev_extState.segment<3>(10) = bias_;
  prev_extState.segment<NOISE_SIZE>(13) =
    Eigen::VectorXd::Zero(NOISE_SIZE);
  // std::cout << "prev_extState: " << prev_extState.transpose() << std::endl;

  prev_extCovariance = ext_covariance_;

  // Eigen::MatrixXd sigma =
  //   (((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
  //   prev_extCovariance).sqrt();
  Eigen::MatrixXd sigma = Eigen::LLT<Eigen::MatrixXd>(
    ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    prev_extCovariance).matrixL();
  // std::cout << "sigma: " << sigma << std::endl;

  std::vector<Eigen::VectorXd> prev_unscentedStates;
  prev_unscentedStates.push_back(prev_extState);
  for (size_t i = 0; i < sigma.cols(); i++) {
    prev_unscentedStates.push_back(prev_extState + sigma.col(i));
    prev_unscentedStates.push_back(prev_extState - sigma.col(i));
    // std::cout << "prev_unscentedStates: " <<
    //   prev_unscentedStates.back().transpose() << std::endl;
  }

  // Search for outliers
  hive::ViveLight clean_msg = msg;
  auto sample_it = clean_msg.samples.begin();
  while (sample_it != clean_msg.samples.end()) {
    if (sample_it->angle > M_PI / 3 || sample_it->angle < -M_PI / 3)
      clean_msg.samples.erase(sample_it);
    else
      sample_it++;
  }

  size_t row = 0;
  Eigen::VectorXd Z(clean_msg.samples.size());
  std::vector<int> sensors;
  for (auto sample : clean_msg.samples) {
    // Put angle in Vector
    Z(row) = sample.angle;
    row++;
    // For later usage
    sensors.push_back(sample.sensor);
  }

  // std::cout << "Z: " << Z.transpose() << std::endl;

  Eigen::MatrixXd R;
  int total_sensors = tracker_.sensors.size();
  if (clean_msg.axis == HORIZONTAL) {
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * HORIZONTAL,
      total_sensors  * HORIZONTAL, total_sensors, total_sensors), sensors);
  } else {
    // Sliced measurement covariance matrix
    R = filter::SliceR(measure_covariance_.block(total_sensors * VERTICAL,
      total_sensors  * VERTICAL, total_sensors, total_sensors), sensors);
  }

  std::vector<Eigen::VectorXd> prev_unscentedSamples;
  for (size_t i = 0; i < prev_unscentedStates.size(); i++) {
    // Format change
    Eigen::Vector3d position = prev_unscentedStates[i].segment<3>(0);
    Eigen::Quaterniond rotation(prev_unscentedStates[i](6),
      prev_unscentedStates[i](7),
      prev_unscentedStates[i](8),
      prev_unscentedStates[i](9));
    rotation.normalize();

    // Choose the model according to the orientation
    if (clean_msg.axis == HORIZONTAL) {
      // Derivative of measurement model
      prev_unscentedSamples.push_back(GetHorizontalZ(
      position, rotation, sensors, tracker_,
      environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_));
    // Choose the model according to the orientation
    } else if (clean_msg.axis == VERTICAL) {
      // Derivative of measurement model
      prev_unscentedSamples.push_back(GetVerticalZ(
      position, rotation, sensors, tracker_,
      environment_.lighthouses[clean_msg.lighthouse],
      lighthouses_[clean_msg.lighthouse], correction_));
    }
    // std::cout << "estZ: " << prev_unscentedSamples.back().transpose() << std::endl;
  }
  // Average and Var and Cov initialization
  Eigen::VectorXd prev_unscentedSample;
  Eigen::MatrixXd prev_unscentedSampleVar;
  Eigen::MatrixXd prev_unscentedCov;

  // E{z}
  prev_unscentedSample = UKF_FACTOR * prev_unscentedSamples[0];
  for (size_t i = 1; i < prev_unscentedSamples.size(); i++) {
    prev_unscentedSample += 0.5 * prev_unscentedSamples[i];
  }
  prev_unscentedSample = 1 / ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    prev_unscentedSample;
  // std::cout << "avgZ: "
    // << prev_unscentedSample.transpose() << std::endl;
  // std::cout << "Z - avgZ: " << (Z - prev_unscentedSample).transpose() << std::endl;

  // Pzz
  prev_unscentedSampleVar = UKF_FACTOR *
    (prev_unscentedSamples[0] - prev_unscentedSample) *
    (prev_unscentedSamples[0] - prev_unscentedSample).transpose();
  for (size_t i = 1; i < prev_unscentedSamples.size(); i++) {
    prev_unscentedSampleVar += 0.5 *
    (prev_unscentedSamples[i] - prev_unscentedSample) *
    (prev_unscentedSamples[i] - prev_unscentedSample).transpose();
  }
  prev_unscentedSampleVar = 1 / ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    prev_unscentedSampleVar;
  // Pvv
  prev_unscentedSampleVar += R;
  // std::cout << "prev_unscentedSampleVar: "
  //   << prev_unscentedSampleVar << std::endl;

  // Pxz
  prev_unscentedCov = UKF_FACTOR *
    (prev_unscentedStates[0] - prev_extState) *
    (prev_unscentedSamples[0] - prev_unscentedSample).transpose();
  for (size_t i = 1; i < prev_unscentedStates.size(); i++) {
    prev_unscentedCov += 0.5 *
      (prev_unscentedStates[i] - prev_extState) *
      (prev_unscentedSamples[i] - prev_unscentedSample).transpose();
  }
  prev_unscentedCov = 1 / ((double)UKF_FACTOR + (double)STATE_SIZE + (double)NOISE_SIZE) *
    prev_unscentedCov;
  // std::cout << "prev_unscentedCov: "
  //   << prev_unscentedCov << std::endl;

  // Kalman Gain
  Eigen::MatrixXd K = prev_unscentedCov * prev_unscentedSampleVar.inverse();

  // std::cout << "K(Z - avgZ): " << (K * (Z - prev_unscentedSample)).transpose() << std::endl;
  // New state
  Eigen::VectorXd next_extState = prev_extState + K * (Z - prev_unscentedSample);
  Eigen::MatrixXd next_extCovariance = prev_extCovariance -
    K * prev_unscentedSampleVar * K.transpose();

  // std::cout << "next_extState: " << next_extState.transpose() << std::endl;
  // std::cout << "next_extCovariance: " << next_extCovariance << std::endl;

  // Save new state
  position_ = next_extState.segment<3>(0);
  velocity_ = next_extState.segment<3>(3);
  rotation_ = Eigen::Quaterniond(next_extState(6),
    next_extState(7),
    next_extState(8),
    next_extState(9)).normalized();
  bias_ = next_extState.segment<3>(10);
  covariance_ = next_extCovariance.block<STATE_SIZE, STATE_SIZE>(0,0);
  ext_covariance_ = next_extCovariance;
  time_ = msg.header.stamp;

  return true;
}

void ViveFilter::PrintState() {
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
  std::cout << velocity_(0) << ", "
    << velocity_(1) << ", "
    << velocity_(2) << ", ";
  Eigen::AngleAxisd AA(rotation_);
  std::cout << AA.axis()(0) * AA.angle() << ", "
    << AA.axis()(1) * AA.angle() << ", "
    << AA.axis()(2) * AA.angle() << std::endl;
  return;
}

// int main(int argc, char ** argv) {
//   ROS_INFO("FILTERING");

//   std::map<std::string, ViveFilter> filter_map;
//   Calibration cal;
//   rosbag::View view;
//   rosbag::Bag rbag;

//   if (argc < 2) {
//     std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
//       << std::endl;
//     return -1;
//   }

//   // Calibration
//   if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
//     ROS_FATAL("Can't find calibration file.");
//     return -1;
//   } else {
//     ROS_INFO("Read calibration file.");
//   }

//   rbag.open(argv[1], rosbag::bagmode::Read);
//   // Lighthouses
//   rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
//   for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
//     const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
//       bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
//     cal.SetLighthouses(*vl);
//   }
//   ROS_INFO("Lighthouses' setup complete.");

//   // Trackers
//   rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
//   for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
//     const hive::ViveCalibrationTrackerArray::ConstPtr vt =
//       bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
//     cal.SetTrackers(*vt);
//     for (auto tr : vt->trackers) {
//       filter_map[tr.serial] = ViveFilter(
//         cal.trackers[tr.serial],
//         cal.lighthouses,
//         cal.environment,
//         1e-2, 1e-6, true,
//         filter::ekf);
//     }
//   }
//   // UKF: 1e-3, 1e-8, true,
//   // (I)EKF: , true,
//   ROS_INFO("Trackers' setup complete.");

//   size_t counter = 0;
//   // Light data
//   std::vector<std::string> run_topics; 
//   run_topics.push_back("/loc/vive/light");
//   run_topics.push_back("/loc/vive/imu/");
//   run_topics.push_back("/loc/vive/imu");
//   rosbag::View view_li(rbag, rosbag::TopicQuery(run_topics));
//   for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
//     const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
//     if (vl != NULL) {
//       std::cout << "LIG  ";// << (int)vl->axis << std::endl;
//       filter_map[vl->header.frame_id].ProcessLight(vl);
//       filter_map[vl->header.frame_id].PrintState();
//       counter++;
//     }
//     const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
//     if (vi != NULL) {
//       std::cout << "IMU  ";// << std::endl;
//       filter_map[vi->header.frame_id].ProcessImu(vi);
//       filter_map[vi->header.frame_id].PrintState();
//       counter++;
//     }
//     // if (counter >= 2600) break;
//   }
//   ROS_INFO("Data processment complete.");

//   return 0;

// }