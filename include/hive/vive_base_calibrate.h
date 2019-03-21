#ifndef VIVE_BASE_CALIBRATE_H
#define VIVE_BASE_CALIBRATE_H

// Includes
#include <ros/ros.h>

// Hive imports
#include <hive/vive.h>
#include <hive/vive_base.h>

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

typedef std::map<std::string, PoseVM> PoseTrackers;
typedef std::map<std::string, PoseTrackers> PoseMap;
typedef std::map<std::string, PoseVM> PoseLighthouses;
typedef std::vector<PoseVM> Poses;
typedef std::map<std::string, Poses> PosesMap;
typedef std::map<std::string, PosesMap> PosesMapMap;
typedef std::map<std::string, std::vector<LightData> > LightDataVector;

struct Sweep {
  LightVec lights;
  std::string lighthouse;
  uint8_t axis;
};
typedef std::vector<Sweep> SweepVec;
typedef std::pair<SweepVec, ImuVec> DataPair;         // pair of Light data and Imu data - change imu
typedef std::map<std::string, DataPair> DataPairMap;         // map of trackers


class BaseCalibrate {
 public:
  explicit BaseCalibrate(Calibration & calibration);

  // Reset the solver
  bool Reset();

  // Process an IMU measurement
  bool AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  bool AddLight(const hive::ViveLight::ConstPtr& msg);

  // Solve the problem based on the assumption tge body of tracker's is still.
  bool Solve();

  // Get the calibration struct
  Calibration GetCalibration();

 private:
  DataPairMap data_pair_map_;   // Input data
  Calibration calibration_;     // Structure that saves all the data
};

#endif // VIVE_BASE_CALIBRATE_H