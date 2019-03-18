/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

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

namespace calibrate {
  double start_pose[6] = {0, 0, 1, 0, 0, 0};
}

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