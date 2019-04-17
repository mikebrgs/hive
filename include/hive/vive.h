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

#ifndef HIVE_VIVE_H_
#define HIVE_VIVE_H_

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS messages
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

// Messages
#include <hive/ViveLight.h>
#include <hive/ViveLightSample.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationLighthouseArray.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationGeneral.h>

// RapidJSON
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"

// Ceres and logging
#include <ceres/ceres.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL C++ includes
#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <cstdio>
#include <vector>
#include <map>

/**
 * \ingroup tools
 */

#define TRACKER_SENSORS_NUMBER 40
#define CORRECTION false

enum AXIS {HORIZONTAL = 0, VERTICAL = 1};
enum LIGHTHOUSE {PHASE = 0,
  TILT = 1,
  GIB_PHASE = 2,
  GIB_MAG = 3,
  CURVE = 4};
enum PARAMETER {POSE = 0,
  EXTRINSICS = 1,
  LIGHTHOUSE = 2,
  RADIUS = 3,
  LH_EXTRINSICS = 4};

struct Transform {
  std::string parent_frame;
  std::string child_frame;
  geometry_msgs::Vector3 translation;
  geometry_msgs::Quaternion rotation;
};

struct TransformStamped {
  Transform transform;
  ros::Time stamp;
};


struct Sensor {
  geometry_msgs::Point position;
  geometry_msgs::Vector3 normal;
};

struct Tracker {
  std::string serial;
  std::map<uint8_t, Sensor> sensors;
  geometry_msgs::Vector3 acc_bias;
  geometry_msgs::Vector3 acc_scale;
  geometry_msgs::Vector3 gyr_bias;
  geometry_msgs::Vector3 gyr_scale;
  // Transform from the imu frame to the light frame (tracker)
  geometry_msgs::Transform imu_transform;
  // Transform from the head frame to the light frame (tracker)
  geometry_msgs::Transform head_transform;
};

struct Motor {
  double phase;
  double tilt;
  double gib_phase;
  double gib_magnitude;
  double curve;
};

struct Lighthouse {
  std::string serial;
  uint8_t id;
  Motor vertical_motor;
  Motor horizontal_motor;
};

typedef std::map<std::string, Lighthouse> LighthouseMap;

struct LightSpecs {
  int timebase_hz;
  int timecenter_ticks;
  int pulsedist_max_ticks;
  int pulselength_min_sync;
  int pulse_in_clear_time;
  int pulse_max_for_sweep;
  int pulse_synctime_offset;
  int pulse_synctime_slack;
};

struct Environment {
  Transform vive;
  Transform offset;  // Body calibration offset
  std::map<std::string, Transform> lighthouses;  // indexed by the serial
  std::map<std::string, Transform> bodies;  // indexed by the serial
  geometry_msgs::Vector3 gravity; // registred gravity in calibration
};

struct Light {
  int sensor_id;
  double timecode;
  double angle;
  double length;
};

struct Imu {
  geometry_msgs::Vector3 accel;
  geometry_msgs::Vector3 gyro;
};

typedef std::vector<Imu> ImuVec;

typedef std::vector<Light> LightVec;

typedef std::pair<Eigen::Vector3d, Eigen::Matrix3d> PoseVM;
typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> PoseVQ;
typedef std::pair<Eigen::Vector3d, Eigen::AngleAxisd> PoseVA;
typedef std::pair<Eigen::Vector3d, Eigen::Vector4d> PoseVV;

typedef std::vector<geometry_msgs::TransformStamped> TFVector;

class Calibration {
 public:
  // Sets the enviroment structure from a calibration message.
  bool SetEnvironment(hive::ViveCalibration const& msg);

  // Pulls the enviroment structure into a calibration message.
  bool GetEnvironment(hive::ViveCalibration * msg);

  // Sets the Light Specification stuff of the laser and flash from a
  // ViveCalibrationGeneral message.
  bool SetLightSpecs(hive::ViveCalibrationGeneral const& msg);

  // Gets the Light Specification stuff of the laser and flash and saves it
  // to the class.
  bool GetLightSpecs(hive::ViveCalibrationGeneral * msg);

  // Sets the lighthouse structure from a ViveCalibrationLighthouseArray message.
  bool SetLighthouses(hive::ViveCalibrationLighthouseArray const& msg);

  // Puts the lighthouse data into a ViveCalibrationLighthouseArray message.
  bool GetLighthouses(hive::ViveCalibrationLighthouseArray * msg);

  // Sets the tracker structure from a ViveCalibrationTrackerArray message.
  bool SetTrackers(hive::ViveCalibrationTrackerArray const& msg);

  // Puts the tracker data into a ViveCalibrationTrackerArray message.
  bool GetTrackers(hive::ViveCalibrationTrackerArray * msg);

  void Print();

  std::map<std::string, Tracker> trackers;
  std::map<std::string, Lighthouse> lighthouses;
  LightSpecs light_specs;
  Environment environment;
};


class ViveUtils {
 public:
  // Write to config file the calibration class
  static bool WriteConfig(std::string file_name,
    Calibration const& calibration);

  // Read from config file the calibration and write to calibration class
  static bool ReadConfig(std::string file_name,
    Calibration * calibration);


  // Broadcast all static trasnforms
  static bool SendTransforms(Calibration const& calibration_data);

  // Get double array from tracker struct - double array previously initialized
  static size_t ConvertExtrinsics(Tracker const& tracker,
    double * extrinsics);

  // Get static transforms in std vector
  static TFVector GetTransforms(Calibration & calibration_data);
};

class JsonParser {
 public:
  // Constructor
  JsonParser(const char * filename);

  // Destructor
  ~JsonParser();

  // Get the update rate of the pose
  double GetRate();

  // Update the calibration structure
  bool GetCalibration(Calibration * calibration);

  // Update the body in the calibration structure
  bool GetBody(Calibration * calibration);
 private:
  rapidjson::Document * document_;
};

// State Machine to handle the solver and calibration
class StateMachine {
public:
  // Constructor
  StateMachine();

  // Destructor
  ~StateMachine();
  
  // Set the initial state
  void SetState(int state);
  
  // Set all the transitions for the state machine
  void AddTransition(int from, int action, int to);
  
  // New transition happening
  void Update(int action);
  
  // Returns the current state
  int GetState();

  void Print();

private:
  int state_;
  std::map<int,std::map<int,int>> machine_;
};

template <typename T>
void hCorrection(T const * position, T const * corrections, T * ang);

template <typename T>
void vCorrection(T const * position, T const * corrections, T * ang);

class ViveModel
{
public:
  ViveModel();

  ViveModel(Eigen::Vector3d position,
  Eigen::Quaterniond rotation);

  ~ViveModel();
  
  void Update(Eigen::Vector3d linear_acceleration,
    Eigen::Vector3d angular_velocity,
    double dT);

  void Update(double dT) ;
  
  sensor_msgs::Imu GetInertialMeasures();
  
  hive::ViveLight GetHorizontalLightMeasures();

  hive::ViveLight GetVerticalLightMeasures();

  void PrintState();

private:
  Tracker tracker_;
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d linear_acceleration_;
  Eigen::Quaterniond rotation_;
  Eigen::Vector3d angular_velocity_;
};

#endif  // VIVE_VIVE_H_