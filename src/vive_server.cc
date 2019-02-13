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

// This package code
#include <hive/vive_general.h>
#include <hive/vive.h>
#include <hive/vive_solve.h>
#include <hive/vive_calibrate.h>
#include <hive/vive_visualization.h>

// Standard C includes
#include <stdio.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_broadcaster.h>

// Standard C++ includes
#include <iostream>

// Services
#include <hive/ViveConfig.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationGeneral.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationLighthouseArray.h>

typedef std::map<std::string, ViveSolve> TrackerMap;
typedef std::map<std::string, Visualization> VisualMap;

size_t counter = 0;

class Hive {
 public:
  Hive(int argc, char ** argv);
  ~Hive();
  void LightCallback(const hive::ViveLight::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void LighthouseCallback(const hive::ViveCalibrationLighthouseArray::ConstPtr& msg);
  void TrackerCallback(const hive::ViveCalibrationTrackerArray::ConstPtr& msg);
  void LightSpecsCallback(const hive::ViveCalibrationGeneral::ConstPtr& msg);
  void TimerCallback(const ros::TimerEvent&);
  void CalibrationCallback(Calibration const& calibration);
  bool ConfigureCallback(hive::ViveConfig::Request & req, hive::ViveConfig::Response & res );
  void Spin();
 private:
  bool ready_;
  std::string calib_file_;              // Name of the calibration file
  Calibration calibration_;        // Structure with all the data
  // Solvers
  std::string solver_;                  // Active solver
  TrackerMap trackers_;                 // Tracker solvers
  VisualMap vive_visualization_;        // visualization objects
  ViveCalibrate calibrator_;            // Calibrator
  // Publishers and Subscribers
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_light_;
  ros::Subscriber sub_lighthouses_;
  ros::Subscriber sub_trackers_;
  ros::Subscriber sub_general_;
  ros::ServiceServer service_;          // Service
  ros::Timer timer_;                    // Tracking timer
  ros::Publisher pub_imu_markers_;      // Imu visualization marker
  ros::Publisher pub_light_markers_;    // light visualization markers
  ros::Publisher pub_tracker_markers_;  // tracker visualization markers
  // State Machine
  StateMachine fsm_;
};

enum STATES {TRACKING = 1, RECORDING = 2, CALIBRATING = 3};
enum EVENTS {START = 1, STOP = 2, DONE = 3};

Hive::Hive(int argc, char ** argv) : calibrator_(std::bind(&Hive::CalibrationCallback, this, std::placeholders::_1)){
  ready_ = false;
  // State machine
  fsm_.AddTransition(TRACKING,START,RECORDING);
  fsm_.AddTransition(TRACKING,STOP,TRACKING);
  fsm_.AddTransition(TRACKING,DONE,TRACKING);
  fsm_.AddTransition(RECORDING,STOP,CALIBRATING);
  fsm_.AddTransition(RECORDING,START,RECORDING);
  fsm_.AddTransition(RECORDING,DONE,RECORDING);
  fsm_.AddTransition(CALIBRATING,DONE,TRACKING);
  fsm_.AddTransition(CALIBRATING,START,CALIBRATING);
  fsm_.AddTransition(CALIBRATING,STOP,CALIBRATING);
  // Initial state
  fsm_.SetState(RECORDING);
  // fsm_.SetState(TRACKING);

  ros::init(argc, argv, "server");
  ros::NodeHandle nh;

  // Subscribers for light measurements
  sub_light_ = nh.subscribe(TOPIC_HIVE_LIGHT, 1000,
    &Hive::LightCallback, this);
  sub_imu_ = nh.subscribe(TOPIC_HIVE_IMU, 1000,
    &Hive::ImuCallback, this);

  // Subscribers to calibration stuff
  sub_lighthouses_ = nh.subscribe(TOPIC_HIVE_LIGHTHOUSES, 1000,
    &Hive::LighthouseCallback, this);
  sub_trackers_ = nh.subscribe(TOPIC_HIVE_TRACKERS, 1000,
    &Hive::TrackerCallback, this);
  sub_general_ = nh.subscribe(TOPIC_HIVE_GENERAL, 1000,
    &Hive::LightSpecsCallback, this);

  // Visualiatization tools
  pub_imu_markers_ = nh.advertise<visualization_msgs::Marker>(
    TOPIC_HIVE_IMU_MARKERS, 1000);
  pub_light_markers_ = nh.advertise<visualization_msgs::MarkerArray>(
    TOPIC_HIVE_LIGHT_MARKERS, 1000);
  pub_tracker_markers_ = nh.advertise<visualization_msgs::MarkerArray>(
    TOPIC_HIVE_TRACKER_MARKERS, 1000);

  // Start JSON parser
  JsonParser jp = JsonParser(HIVE_CONFIG_FILE);

  // Periodic timer to query and send pose
  timer_ = nh.createTimer(ros::Rate(jp.GetRate()),
      &Hive::TimerCallback, this, false, true);

  // Calibration service
  service_ = nh.advertiseService(SERVICE_HIVE_CONFIG,
      &Hive::ConfigureCallback, this);


  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &calibration_)) {
    jp.GetCalibration(&calibration_);
  } else {
    jp.GetBody(&calibration_);
  }

  calibrator_.Reset();
  calibrator_.Initialize(calibration_);

  ViveUtils::SendTransforms(calibration_);
  ready_ = true;

  return;
}

Hive::~Hive() {
  // pass
}

void Hive::LightCallback(const hive::ViveLight::ConstPtr& msg) {
  // Check the current state of the system
  counter++;
  switch(fsm_.GetState()) {
    case TRACKING:
      // In the case where the calibration is not available
      if (!ready_) return;
      // Check if tracker is registred
      if (trackers_.find(msg->header.frame_id) == trackers_.end()) {
        ROS_FATAL("Can't find tracker");
        return;
      }
      // Add data to solver
      trackers_[msg->header.frame_id].ProcessLight(msg);
      vive_visualization_[msg->header.frame_id].AddLight(msg);
      break;
    case RECORDING:
      calibrator_.AddLight(msg);
      // if (counter == 100) {
        // calibrator_.Initialize(calibration_);
        // calibrator_.Solve();
        // fsm_.Update(DONE);
        // exit(0);
      // }
      break;
    default:
      break;
  }
}

void Hive::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  switch(fsm_.GetState()) {
    case TRACKING:
      // In the case where the calibration is not available
      if (!ready_) return;
      // Check if tracker is registred
      if (trackers_.find(msg->header.frame_id) == trackers_.end()) {
        ROS_FATAL("Can't find tracker");
        return;
      }
      // Add data to solver
      trackers_[msg->header.frame_id].ProcessImu(msg);
      break;
    case RECORDING:
      calibrator_.AddImu(msg);
      break;
    default:
      break;
  }
}

void Hive::LighthouseCallback(const hive::ViveCalibrationLighthouseArray::ConstPtr& msg) {
  calibration_.SetLighthouses(*msg);
  for (auto tr_it = trackers_.begin(); tr_it !=  trackers_.end(); tr_it++) {
    // Update Solver
    tr_it->second.Update(calibration_.lighthouses);
  }
  calibrator_.Update(calibration_.lighthouses);
}

void Hive::TrackerCallback(const hive::ViveCalibrationTrackerArray::ConstPtr& msg) {
  calibration_.SetTrackers(*msg);
  for (std::map<std::string, Tracker>::const_iterator tr_it = calibration_.trackers.begin();
    tr_it != calibration_.trackers.end(); tr_it++) {
    // Update Solver
    trackers_[tr_it->first].Initialize(calibration_.environment,
      tr_it->second);
    // Update Visualization tools
    vive_visualization_[tr_it->first].Initialize(tr_it->second, trackers_.size()-1);
  }
}

void Hive::LightSpecsCallback(const hive::ViveCalibrationGeneral::ConstPtr& msg) {
  calibration_.SetLightSpecs(*msg);
}

void Hive::TimerCallback(const ros::TimerEvent&) {
  // Ignore if not in tracking mode
  if (fsm_.GetState() != TRACKING) return;
  // Iterate over all trackers that we are solving for, and send their pose
  for (TrackerMap::iterator tr_it = trackers_.begin();
    tr_it != trackers_.end(); tr_it++) {
    // Get the transform
    geometry_msgs::TransformStamped tf;
    if (tr_it->second.GetTransform(tf)) {
      static tf2_ros::TransformBroadcaster br;
      br.sendTransform(tf);
      // IMU
      visualization_msgs::Marker arrow;
      if (vive_visualization_[tr_it->first].GetImu(&arrow)) {
        pub_imu_markers_.publish(arrow);
      }
      // LIGHTS
      visualization_msgs::MarkerArray directions;
      if (vive_visualization_[tr_it->first].GetLight(&directions)) {
        pub_light_markers_.publish(directions);
      }
      // SENSORS
      visualization_msgs::MarkerArray sensors;
      if (vive_visualization_[tr_it->first].GetSensors(&sensors)) {
        pub_tracker_markers_.publish(sensors);
      }
    }
  }
  return;
}

// Called back when the calibration procedure completes
void Hive::CalibrationCallback(Calibration const& calibration) {
  ViveUtils::WriteConfig(HIVE_CALIBRATION_FILE, calibration);
  ViveUtils::SendTransforms(calibration);
  // Set solvers with the right parameters
  for (TrackerMap::iterator tr_it = trackers_.begin();
    tr_it != trackers_.end(); tr_it++) {
    tr_it->second.Update(calibration.environment);
  }
  calibration_ = calibration;
  calibrator_.Reset();
  ready_ = true;
  fsm_.Update(DONE);
  std::cout << "Calibration Saved" << std::endl;
  // exit(0);
  return;
}

// Configuration call from the vive_tool
bool Hive::ConfigureCallback(hive::ViveConfig::Request & req,
                        hive::ViveConfig::Response & res ) {
  switch (req.action) {
  case hive::ViveConfig::Request::START:
    if (fsm_.GetState() == TRACKING) {
      fsm_.Update(START);
      std::cout << "RECORDING " << fsm_.GetState() << std::endl;
      calibrator_.Reset();
      // calibrator_.Initialize(calibration_);
    }
    res.success = true;
    res.status = std::to_string(fsm_.GetState());
    break;
  case hive::ViveConfig::Request::STOP:
    if (fsm_.GetState() == RECORDING) {
      std::cout << "CALIBRATING " << fsm_.GetState() << std::endl;
      fsm_.Update(STOP);
      calibrator_.Initialize(calibration_);
      calibrator_.Solve();
    }
    res.success = true;
    res.status = std::to_string(fsm_.GetState());
    break;
  default:
    return false;
  }
  return true;
}

void Hive::Spin() {
  ROS_INFO("Spinning");
  ros::spin();
}

int main(int argc, char **argv) {
  // Initializing Hive
  Hive hive = Hive(argc, argv);
  hive.Spin();
  return 0;
}