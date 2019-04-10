// The vive pose is fixed and we try to improve the conection
// between lighthouses
#ifndef HIVE_VIVE_REFINE_H_
#define HIVE_VIVE_REFINE_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive_general.h>
#include <hive/vive_solve.h>
#include <hive/vive.h>

// Incoming measurements
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <hive/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// C++11 includes
#include <utility>
#include <vector>
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <string>

// Internal datatypes
namespace refine {
  typedef std::vector<hive::ViveLight> SweepVec;
  typedef std::vector<sensor_msgs::Imu> ImuVec;
  typedef std::pair<SweepVec, ImuVec> DataPair;         // pair of Light data and Imu data - change imu
  typedef std::map<std::string, DataPair> DataMap;         // map of trackers
} // namespace refine

using namespace refine;

class Refinery {
private:
  Calibration calibration_;
  DataMap data_;
  // Parameters
  bool correction_;
  double smoothing_;
public:
  // Initialize
  Refinery(Calibration & calibration);
  Refinery(Calibration & calibration, bool correction);
  Refinery(Calibration & calibration, bool correction, double smoothing);

  // Destroy
  ~Refinery();

  // Process an IMU measurement
  bool AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  bool AddLight(const hive::ViveLight::ConstPtr& msg);

  // Solve the problem based on the assumption tge body of tracker's is still.
  bool Solve();
};

class SmoothingCost {
public:
  // Constructor to pass data
  // A good smoothing factor is 0.1, but it may be changed
  explicit SmoothingCost(double smoothing);

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
};

#endif