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
#include <sensor_msgs/Imu.h>
#include <hive/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <string>


class PoseGraph : public Solver {
public:
  // Constructor
  PoseGraph();
  PoseGraph(size_t window);
  // Destructor
  ~PoseGraph();
  // New light data
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // New Imu data
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Get the tracker's pose
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Solve the problem
  bool Solve();
  // Prinst stuff
  void PrintState();
private:
  // The pose
  geometry_msgs::TransformStamped pose_;
  // Light data
  std::vector<hive::ViveLight> light_data_;
  // Inertial data
  std::vector<sensor_msgs::Imu> imu_data_;
  // Calibrated environment
  Environment environment_;
  // Tracker
  Tracker tracker_;
  // Lighthouses specs
  std::map<std::string, Lighthouse> lighthouses_;
  // Correction
  bool correction_;
  // Optimization window
  size_t window_;
  // Validity
  bool valid_;
};

// Inertial cost function
class InertialCost {
public:
  InertialCost(sensor_msgs::Imu imu,
    geometry_msgs::TransformStamped prev_vTl,
    geometry_msgs::TransformStamped next_vTl,
    double time_step);
  ~InertialCost();
  template <typename T> bool operator()(const T* const prev_lTt,
    const T* const next_lTt,
    T * residual) const;
private:
  // Inertial data
  sensor_msgs::Imu imu_;
  // Light data
  hive::ViveLight prev_, next_;
  // Environment transforms
  geometry_msgs::TransformStamped prev_vTl_, next_vTl_;
  // Time step
  double time_step_;
};

#endif // HIVE_VIVE_PGO