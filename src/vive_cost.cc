#include <hive/vive_cost.h>

ViveHorizontalCost::ViveHorizontalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
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
  vPl << T(lTv_.translation.x),
    T(lTv_.translation.y),
    T(lTv_.translation.z);
  Eigen::Quaternion<T> vQl(
    T(lTv_.rotation.w),
    T(lTv_.rotation.x),
    T(lTv_.rotation.y),
    T(lTv_.rotation.z));
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
  geometry_msgs::Transform lTv,
  Tracker tracker,
  Motor lighthouse,
  bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
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
  vPl << T(lTv_.translation.x),
    T(lTv_.translation.y),
    T(lTv_.translation.z);
  Eigen::Quaternion<T> vQl(
    T(lTv_.rotation.w),
    T(lTv_.rotation.x),
    T(lTv_.rotation.y),
    T(lTv_.rotation.z));
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
