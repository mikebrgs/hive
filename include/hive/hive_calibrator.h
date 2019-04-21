#ifndef HIVE_HIVE_CALIBRATE_H_
#define HIVE_HIVE_CALIBRATE_H_

#include <ros/ros.h>

#include <hive/vive_solve.h>
#include <hive/vive.h>

// Incoming measurements
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationTrackerArray2.h>
#include <hive/ViveCalibrationLighthouseArray.h>

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

// Internal datatypes
namespace calibrate {
  struct Sweep {
    LightVec lights;
    std::string lighthouse;
    uint8_t axis;
  };
  typedef std::vector<Sweep> SweepVec;
  typedef std::vector<sensor_msgs::Imu> ImuVec;
  typedef std::pair<SweepVec, ImuVec> DataPair;         // pair of Light data and Imu data - change imu
  typedef std::map<std::string, DataPair> DataPairMap;         // map of trackers
} // namespace calibrate

using namespace calibrate;

class ViveCalibrate {
 public:

  // Constructor
  ViveCalibrate(Calibration & calibration,
    bool correction);

  // Destructor
  ~ViveCalibrate();

  // Reset the solver
  bool Reset();

  // Process an IMU measurement
  bool AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  bool AddLight(const hive::ViveLight::ConstPtr& msg);

  // Solve the problem based on the assumption tge body of tracker's is still.
  bool Solve();

  // New tracker data
  bool Update(const hive::ViveCalibrationTrackerArray2::ConstPtr& msg);

  // Old tracker data
  bool Update(const hive::ViveCalibrationTrackerArray::ConstPtr& msg);

  // New lighthouse data
  bool Update(const hive::ViveCalibrationLighthouseArray::ConstPtr& msg);

  // Debug method
  void Print();

  // Get the calibration struct
  Calibration GetCalibration();

  // // Thread that solves
  // static void WorkerThread(CallbackFn cb,
  //   std::mutex * calibration_mutex,
  //   DataPairMap data_pair_map,
  //   Calibration calibration);

 private:
  std::vector<sensor_msgs::Imu> imu_data_;
  std::vector<hive::ViveLight> light_data_;
  DataPairMap data_pair_map_;   // Input data
  bool correction_;
  bool active_;                 // If the calibration procedure is active
  Calibration calibration_;     // Structure that saves all the data
};

#endif  // VIVE_HIVE_CALIBRATE_H_