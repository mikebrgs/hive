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
    geometry_msgs::Transform lTv, // vive to lighthouse
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveHorizontalCost();
  // Ceres operator
  template <typename T> bool operator()(const T* const * parameters,
    T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lTv_, tTi_;
};

// Light cost - Cost using the poses from the imu frame to the vive frame
class ViveVerticalCost
{
public:
  // Constructor
  ViveVerticalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv, // Vive to lighthouse
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveVerticalCost();
  // Ceres operator
  template <typename T> bool operator()(const T* const * parameters,
    T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lTv_, tTi_;
};

#endif // HIVE_VIVE_COST_H_