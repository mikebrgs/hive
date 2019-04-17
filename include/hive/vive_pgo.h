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


class PoseGraph : public Solver {
public:
  // Constructor
  PoseGraph(size_t window);
  // Destructor
  ~PoseGraph();
  // New light data
  void ProcessLight(const sensor_msgs::Imu::ConstPtr& msg);
  // New Imu data
  void ProcessImu(const hive::ViveLight::ConstPtr& msg);
  // Get the tracker's pose
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Prinst stuff
  void PrintState();
private:
  // Light data
  std::vector<hive::ViveLight> light_data_;
  // Inertial data
  std::vector<sensor_msgs::> imu_data_;
  // Calibrated environment
  Environment environment_;
  // Tracker
  Tracker tracker_;
  // Lighthouses specs
  std::map<std::string, Lighthouse> lighthouses_;
  // Correction
  bool correction_;
};

#endif