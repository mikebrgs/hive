#ifndef HIVE_VIVE_FILTER_H_
#define HIVE_VIVE_FILTER_H_

#include <ros/ros.h>

#include <hive/vive.h>

// Incoming measurements
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STD C includes
#include <math.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STD C++ includes
#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <string>

typedef struct State
{
  double[3] position;
  double[3] velocity;
  double[4] orientation;
  double[3] bias;
} State;

class ViveFilter
{
  // Constructor
  ViveFilter();

  // Destructor
  ~ViveFilter();

  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);

  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped &msg);

  // // Set all the necessary parameters
  // bool Initialize(Tracker const& tracker);

  // // Set all the necessary parameters
  // bool Initialize(Environment const& environment,
  //   Tracker const& tracker);

  // // Updates the calibration file in case it gets updated
  // bool Update(Environment const& environment);

  // // Update lighthouse extrinsics
  // bool Update(LighthouseMap const& lh_extrinsics);

 private:
  // Poses of tracker relatively to lighthouses
  // std::map<std::string, PoseVQ> poses;

  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d w_bias_;
  Eigen::Vector3d gravity_;
  PoseVQ imu_transform_;
  // std::map<std::string, SolvedPose> poses_;
  // SolvedPose tracker_pose_;
  // Environment environment_;
  // LightData observations_;
  // Extrinsics extrinsics_;
  std::mutex * solveMutex_;
  Tracker tracker_;
  LighthouseMap lh_extrinsics_;
};

#endif  // HIVE_VIVE_FILTER_H_