#ifndef HIVE_VIVE_EKF_H_
#define HIVE_VIVE_EKF_H_

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

class ViveEKF : public Solver{
public:
  // Constructor
  ViveEKF(); // Initial template
  ViveEKF(geometry_msgs::TransformStamped & pose,
    Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double ** model_covariance, // time update
    double ** measure_covariance, // measurements
    Solver * solver,
    bool correction);
  // Destructor
  ~ViveEKF();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Temporary
  void PrintState();
// private: // temporary
  // Internal method
  bool Predict(const sensor_msgs::Imu & msg);
  // Internal method
  bool Update(const hive::ViveLight & msg);
  // Validity
  bool Valid();
private:
  // State
  Eigen::Vector3d position_;
  // geometry_msgs::Vector3 position_;
  Eigen::Vector3d velocity_;
  // geometry_msgs::Vector3 velocity_;
  Eigen::Quaterniond rotation_;
  // geometry_msgs::Quaternion rotation_;
  Eigen::Vector3d bias_;
  // geometry_msgs::Vector3 bias_;
  ros::Time time_;
  // Covatiances
  Eigen::MatrixXd model_covariance_;
  // double * tu_covariance;
  Eigen::MatrixXd measure_covariance_;
  // double * mu_covari>ance;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specifications
  std::map<std::string, Lighthouse> lighthouses_;
  // Calibration environment
  Environment environment_;
  // The tracker it's solving for
  Tracker tracker_;
  // If the correction parameters are to be used
  bool correction_;
  // Validity of current state
  bool valid_;
  // Optimizing solver
  Solver * solver_;
};

#endif  // HIVE_VIVE_EKF_H_