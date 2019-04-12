#ifndef HIVE_VIVE_PGO
#define HIVE_VIVE_PGO

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive_general.h>
#include <hive/vive_solve.h>
#include <hive/vive.h>

// Hive msgs
#include <hive/ViveLight.h>
#include <sensor_msgs::Imu.h>
#include <hive/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <string>


class PoseGraph {
public:
  PoseGraph();
  ~PoseGraph();
  bool AddLight();
  bool AddImu();
  bool Solve();
private:
  std::vector<hive::ViveLight> light_data_;
  std::vector<sensor_msgs::> imu_data_;
};

#endif