#ifndef HIVE_VIVE_BASE_SOLVE_H_
#define HIVE_VIVE_BASE_SOLVE_H_

// C standard includes
#include <stdlib.h>

// C++ standard includes
#include <iostream>
#include <mutex>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS messages
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>


// Hive includes
#include <hive/vive_general.h>
#include <hive/vive_solver.h>
#include "hive/vive.h"

// namespace base{
//   double start_pose[6] = {0, 0, 1, 0, 0, 0};
// }

struct LightVecStamped {
  LightVec lights;
  ros::Time stamp;
};

struct AxisLightVec {
  std::string lighthouse;
  // axis - observation
  std::map<uint8_t, LightVecStamped> axis;
};

// Lighthouse - both observations
typedef std::map<std::string, AxisLightVec> LightData;

struct Extrinsics {
  double positions[3 * TRACKER_SENSORS_NUMBER];
  double normals[3 * TRACKER_SENSORS_NUMBER];
  double radius;
  size_t size;
};

struct SolvedPose {
  double transform[6];
  std::string lighthouse;
  bool valid;
  ros::Time stamp;
};


class BaseSolve : public Solver {
public:
  BaseSolve();
  BaseSolve(Environment environment,
    Tracker tracker);
  ~BaseSolve();
  // bool Initialize(Environment const& environment,
    // Tracker const& tracker);
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  bool GetTransform(geometry_msgs::TransformStamped &msg);
private:
  std::map<std::string, SolvedPose> poses_;
  SolvedPose tracker_pose_;
  Environment environment_;
  LightData observations_;
  Extrinsics extrinsics_;
  Tracker tracker_;
};

typedef std::map<std::string, BaseSolve> BaseMap;

#endif // VIVE_BASE_SOLVE_H_