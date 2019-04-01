// The vive pose is fixed and we try to improve the conection
// between lighthouses
#ifndef HIVE_VIVE_REFINE_H_
#define HIVE_VIVE_REFINE_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
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

class Refinery {
private:
  Calibration calibration_;
public:
  // Initialize
  Refinery(Calibration & calibration);

  // Destroy
  ~Refinery();

  // Process an IMU measurement
  bool AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  bool AddLight(const hive::ViveLight::ConstPtr& msg);

  // Solve the problem based on the assumption tge body of tracker's is still.
  bool Solve();
};

#endif