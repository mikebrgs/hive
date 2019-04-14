#ifndef HIVE_VIVE_FILTER_H_
#define HIVE_VIVE_FILTER_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive.h>
#include <hive/vive_solver.h>

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
#include <mutex>
#include <vector>
#include <thread>
#include <string>

// typedef struct State
// {
//   double position[3];
//   double velocity[3];
//   double orientation[4];
//   double bias[3];
// } State;

class ViveFilter : public Solver{
public:
  // Constructor
  ViveFilter(); // Initial template
  ViveFilter(geometry_msgs::TransformStamped & pose,
    Tracker & tracker,
    std::vector<Lighthouse> & lighthouses,
    Environment & environment,
    double * tu_covariance, // concatenated by row -- time update
    double * mu_covariance, // concatenated by row -- measurements
    bool correction);
  // Destructor
  ~ViveFilter();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
// private: // temporary
  // Internal method
  bool Predict(const sensor_msgs::Imu & msg);
  // Internal method
  bool Update(const hive::ViveLight & msg);
private:
  // State
  geometry_msgs::Vector3 position_;
  geometry_msgs::Vector3 velocity_;
  geometry_msgs::Quaternion rotation_;
  geometry_msgs::Vector3 bias_;
  // Covatiances
  double * tu_covariance;
  double * mu_covariance;
  // Other elements
  std::vector<Lighthouse> lighthouses_;
  Environment environment_; // TODO include gravity in Environment
  Tracker tracker_;
  bool correction_;
  // Temp
  geometry_msgs::Vector3 gravity_;
};

#endif  // HIVE_VIVE_FILTER_H_