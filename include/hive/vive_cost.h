#ifndef HIVE_VIVE_COST_H_
#define HIVE_VIVE_COST_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive.h>

// Incoming measurements
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STD C includes
#include <math.h>

// STD C++ includes
#include <map>
#include <vector>
#include <string>


// Light cost - Cost using the poses from the imu frame to the vive frame
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

// Inertial cost function
class InertialCost {
public:
  InertialCost(sensor_msgs::Imu imu,
    geometry_msgs::Vector3 gravity,
    geometry_msgs::Vector3 gyr_bias,
    double time_step,
    double trust_weight,
    bool verbose = false);
  ~InertialCost();
  template <typename T> bool operator()(const T* const prev_vTi,
    const T* const next_vTi,
    T * residual) const;
private:
  // Inertial data
  sensor_msgs::Imu imu_;
  // Light data
  hive::ViveLight prev_, next_;
  // Gravity
  geometry_msgs::Vector3 gravity_, gyr_bias_;
  // Time step and weight
  double time_step_, trust_weight_;
  // Other
  bool verbose_;
};

class SmoothingCost {
public:
  // Constructor to pass data
  // A good smoothing factor is 0.1, but it may be changed
  explicit SmoothingCost(double smoothing,
    double rotation_factor);

  // Function for ceres solver with parameters (different frames)
  template <typename T> bool operator()(const T* const prev_lTt,
    const T* const prev_vTl,
    const T* const next_lTt,
    const T* const next_vTl,
    T * residual) const;

  // Function for ceres solver with parameters (same frame)
  template <typename T> bool operator()(const T* const prev_lTt,
    const T* const next_lTt,
    const T* const vTl,
    T * residual) const;

private:
  double smoothing_;
  double rotation_factor_;
};

// Light cost - Cost using the poses from the imu frame to the vive frame
class ViveCalibrationHorizontalCost
{
public:
  // Constructor
  ViveCalibrationHorizontalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveCalibrationHorizontalCost();
  // Ceres operator - First parameters are Vive Pose, second are lighthouse pose
  template <typename T>
  bool operator()(const T* const * parameters, T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
};

// Light cost - Cost using the poses from the imu frame to the vive frame
class ViveCalibrationVerticalCost
{
public:
  // Constructor
  ViveCalibrationVerticalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveCalibrationVerticalCost();
  // Ceres operator - First parameters are Vive Pose, second are lighthouse pose
  template <typename T>
  bool operator()(const T* const * parameters, T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
};

/*  IMPLEMENTATION  */

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

InertialCost::InertialCost(sensor_msgs::Imu imu,
  geometry_msgs::Vector3 gravity,
  geometry_msgs::Vector3 gyr_bias,
  double time_step,
  double trust_weight,
  bool verbose) {
  imu_ = imu;
  gravity_ = gravity;
  time_step_ = time_step;
  trust_weight_ = trust_weight;
  gyr_bias_ = gyr_bias;
  verbose_ = verbose;
  return;
}

InertialCost::~InertialCost() {
  // Nothing happens
  return;
}

template <typename T>
bool InertialCost::operator()(const T* const prev_vTi,
  const T* const next_vTi,
  T * residual) const {

  // prev_vTt's state
  // Position
  Eigen::Matrix<T,3,1> prev_vPi;
  prev_vPi << prev_vTi[0],
    prev_vTi[1],
    prev_vTi[2];
  // Velocity
  Eigen::Matrix<T,3,1> prev_vVi;
  prev_vVi << prev_vTi[3],
    prev_vTi[4],
    prev_vTi[5];
  // Angle axis in vector
  Eigen::Matrix<T,3,1> prev_vAi;
  prev_vAi << prev_vTi[6],
    prev_vTi[7],
    prev_vTi[8];
  // Angle axis eigen structure
  Eigen::AngleAxis<T> prev_vAAi(prev_vAi.norm(),
    prev_vAi.normalized());
  // Eigen Quaternion
  Eigen::Quaternion<T> prev_vQi(prev_vAAi);
  // Rotation matrix
  Eigen::Matrix<T,3,3> prev_vRi;// = prev_vQi.toRotationMatrix();
  ceres::AngleAxisToRotationMatrix(&prev_vTi[6], prev_vRi.data());

  // next_vTt's state
  // Position
  Eigen::Matrix<T,3,1> next_vPi;
  next_vPi << next_vTi[0],
    next_vTi[1],
    next_vTi[2];
  // Velocity
  Eigen::Matrix<T,3,1> next_vVi;
  next_vVi << next_vTi[3],
    next_vTi[4],
    next_vTi[5];
  // Angle axis in vector
  Eigen::Matrix<T,3,1> next_vAi;
  next_vAi << next_vTi[6],
    next_vTi[7],
    next_vTi[8];
  // Angle axis eigen structure
  Eigen::AngleAxis<T> next_vAAi(next_vAi.norm(),
    next_vAi.normalized());
  // Eigen Quaternion
  Eigen::Quaternion<T> next_vQi(next_vAAi);
  // Rotation matrix
  Eigen::Matrix<T,3,3> next_vRi;// = next_vQi.toRotationMatrix();
  ceres::AngleAxisToRotationMatrix(&next_vTi[6], next_vRi.data());

  // Inertial measurements
  Eigen::Matrix<T,3,1> iW;
  iW << T(imu_.angular_velocity.x),
    T(imu_.angular_velocity.y),
    T(imu_.angular_velocity.z);
  Eigen::Matrix<T,3,1> iA;
  iA << T(imu_.linear_acceleration.x),
    T(imu_.linear_acceleration.y),
    T(imu_.linear_acceleration.z);

  // Inertial bias
  Eigen::Matrix<T,3,1> iB;
  iB << T(gyr_bias_.x),
    T(gyr_bias_.y),
    T(gyr_bias_.z);

  // Gravity
  Eigen::Matrix<T,3,1> vG;
  vG << T(gravity_.x),
    T(gravity_.y),
    T(gravity_.z);

  // Inertial predictions
  // Position prediction
  Eigen::Matrix<T,3,1> est_vPi;
  est_vPi = prev_vPi + T(time_step_) * prev_vVi;// + T(0.5 * time_step_ * time_step_) * (vG - prev_vRi * iA);
  // Velocity prediction
  Eigen::Matrix<T,3,1> est_vVi;
  est_vVi = prev_vVi + T(time_step_) * (vG - (prev_vRi * iA));
  if (verbose_) {
    std::cout << "vA" << " "
      << (prev_vRi * iA)(0) << ", "
      << (prev_vRi * iA)(1) << ", "
      << (prev_vRi * iA)(2) << std::endl;
    std::cout << "vG" << " "
      << vG(0) << ", "
      << vG(1) << ", "
      << vG(2) << std::endl;
  }
  // std::cout << (prev_vRi * iA)(0) << ", " << (prev_vRi * iA)(1) << ", " << (prev_vRi * iA)(2) << std::endl;
  // Quaternion derivative matrix
  Eigen::Matrix<T,4,3> Omega;
  Omega << -prev_vQi.x(), -prev_vQi.y(), -prev_vQi.z(),
    prev_vQi.w(), -prev_vQi.z(), prev_vQi.y(),
    prev_vQi.z(), prev_vQi.w(), -prev_vQi.x(),
    -prev_vQi.y(), prev_vQi.x(), prev_vQi.w();
  // Temporary previous orientation in vector
  Eigen::Matrix<T,4,1> trev_vQi; // temporary previous
  trev_vQi << prev_vQi.w(),
    prev_vQi.x(),
    prev_vQi.y(),
    prev_vQi.z();
  Eigen::Matrix<T,4,1> text_vQi; // temporary next
  // Temporary next orientation in vector
  text_vQi = trev_vQi + T(time_step_) * T(0.5) * Omega * (iW - iB);
  // text_vQi.normalize(); // maybe remove
  // Fix small errors
  Eigen::Quaternion<T> est_vQi(text_vQi(0),
    text_vQi(1),
    text_vQi(2),
    text_vQi(3));
  Eigen::Matrix<T,3,3> est_vRi = est_vQi.toRotationMatrix();

  // Estimated from inertia vs next
  Eigen::Matrix<T,3,1> dP;
  Eigen::Matrix<T,3,1> dV;
  Eigen::Matrix<T,3,3> dR;
  dP = next_vPi - est_vPi;
  dV = next_vVi - est_vVi;
  dR = next_vRi.transpose() * est_vRi;

  // Position cost
  residual[0] = T(trust_weight_) * dP(0);
  residual[1] = T(trust_weight_) * dP(1);
  residual[2] = T(trust_weight_) * dP(2);
  // Velocity cost
  // residual[3] = prev_vPi(0) - next_vPi(0);
  // residual[4] = prev_vPi(1) - next_vPi(1);
  // residual[5] = prev_vPi(2) - next_vPi(2);
  residual[3] = T(trust_weight_) * dV(0);
  residual[4] = T(trust_weight_) * dV(1);
  residual[5] = T(trust_weight_) * dV(2);
  // Orientation cost
  T aa[3];
  ceres::RotationMatrixToAngleAxis(dR.data(), aa);
  // std::cout << aa[0] << ", " << aa[1] << ", " << aa[2] << std::endl;
  residual[6] = T(trust_weight_) *
    (aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); // WATCH OUT FOR THIS
    // Watch out for bias
  return true;
}

// Constructor to pass data
SmoothingCost::SmoothingCost(double smoothing,
  double rotation_factor) {
  smoothing_ = smoothing;
  rotation_factor_ = rotation_factor;
}
// Function for ceres solver with parameters
template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
  const T* const prev_vTl,
  const T* const next_lTt,
  const T* const next_vTl,
  T * residual) const {
  // cost function here

  // Frame conversion
  Eigen::Matrix<T, 3, 1> prev_lPt;
  prev_lPt << prev_lTt[0],
    prev_lTt[1],
    prev_lTt[2];
  Eigen::Matrix<T, 3, 1> next_lPt;
  next_lPt << next_lTt[0],
    next_lTt[1],
    next_lTt[2];
  Eigen::Matrix<T, 3, 3> prev_lRt;
  ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
  Eigen::Matrix<T, 3, 3> next_lRt;
  ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

  Eigen::Matrix<T, 3, 1> prev_vPl;
  prev_vPl << prev_vTl[0],
    prev_vTl[1],
    prev_vTl[2];
  Eigen::Matrix<T, 3, 1> next_vPl;
  next_vPl << next_vTl[0],
    next_vTl[1],
    next_vTl[2];
  Eigen::Matrix<T, 3, 3> prev_vRl;
  ceres::AngleAxisToRotationMatrix(&prev_vTl[3], prev_vRl.data());
  Eigen::Matrix<T, 3, 3> next_vRl;
  ceres::AngleAxisToRotationMatrix(&next_vTl[3], next_vRl.data());

  Eigen::Matrix<T, 3, 1> prev_vPt = prev_vRl * prev_lPt + prev_vPl;
  Eigen::Matrix<T, 3, 3> prev_vRt = prev_vRl * prev_lRt;

  Eigen::Matrix<T, 3, 1> next_vPt = next_vRl * next_lPt + next_vPl;
  Eigen::Matrix<T, 3, 3> next_vRt = next_vRl * next_lRt;

  // // Translation cost with smoothing
  residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
  residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
  residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
  // // Rotation cost with smoothing
  T aa[3];
  Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
  ceres::RotationMatrixToAngleAxis(R.data(), aa);
  residual[3] = T(rotation_factor_) * T(smoothing_)*
    sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
  return true;
}

template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
  const T* const next_lTt,
  const T* const vTl,
  T * residual) const {
  // Frame conversion
  Eigen::Matrix<T, 3, 1> prev_lPt;
  prev_lPt << prev_lTt[0],
    prev_lTt[1],
    prev_lTt[2];
  Eigen::Matrix<T, 3, 1> next_lPt;
  next_lPt << next_lTt[0],
    next_lTt[1],
    next_lTt[2];
  Eigen::Matrix<T, 3, 3> prev_lRt;
  ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
  Eigen::Matrix<T, 3, 3> next_lRt;
  ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

  Eigen::Matrix<T, 3, 1> vPl;
  vPl << vTl[0],
    vTl[1],
    vTl[2];
  Eigen::Matrix<T, 3, 3> vRl;
  ceres::AngleAxisToRotationMatrix(&vTl[3], vRl.data());

  Eigen::Matrix<T, 3, 1> prev_vPt = vRl * prev_lPt + vPl;
  Eigen::Matrix<T, 3, 3> prev_vRt = vRl * prev_lRt;

  Eigen::Matrix<T, 3, 1> next_vPt = vRl * next_lPt + vPl;
  Eigen::Matrix<T, 3, 3> next_vRt = vRl * next_lRt;

  // // Translation cost with smoothing
  residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
  residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
  residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
  // // Rotation cost with smoothing
  T aa[3];
  Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
  ceres::RotationMatrixToAngleAxis(R.data(), aa);
  residual[3] = T(rotation_factor_) * T(smoothing_) *
    sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
  return true;
}

ViveCalibrationHorizontalCost::ViveCalibrationHorizontalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  return;
}

ViveCalibrationHorizontalCost::~ViveCalibrationHorizontalCost() {
  // Do nothing
  return;
}

template <typename T>
bool ViveCalibrationHorizontalCost::operator()(const T* const * parameters,
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
  vPl << parameters[1][0],
    parameters[1][1],
    parameters[1][2];
  Eigen::Matrix<T, 3, 3> vRl;
  ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

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

ViveCalibrationVerticalCost::ViveCalibrationVerticalCost(hive::ViveLight data,
  Tracker tracker,
  Motor lighthouse,
  bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  return;
}

ViveCalibrationVerticalCost::~ViveCalibrationVerticalCost() {
  // Do nothing
  return;
}

template <typename T>
bool ViveCalibrationVerticalCost::operator()(const T* const * parameters,
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
  vPl << parameters[1][0],
    parameters[1][1],
    parameters[1][2];
  Eigen::Matrix<T, 3, 3> vRl;
  ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

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


#endif // HIVE_VIVE_COST_H_