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

// ROS includes
#include <hive/vive_solve.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <thread>

namespace solve{
  double start_pose[6] = {0, 0, 1, 0, 0, 0};
}

ViveSolve::ViveSolve() {
  solveMutex_ = new std::mutex();
  // Do nothing
}

ViveSolve::~ViveSolve() {
  delete solveMutex_;
  // Do nothing
}

void ViveSolve::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // Do something
  if (msg == NULL) {
    return;
  }
}

void ViveSolve::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }

  // Check if this is a new lighthouse
  if (poses_.find(msg->lighthouse) == poses_.end()) {
    // Set structures
    observations_[msg->lighthouse].lighthouse = msg->lighthouse;
  }

  // Clear the axis with old data - should only do this if the new data is good
  observations_[msg->lighthouse].axis[msg->axis].lights.clear();

  // Iterate all sweep data
  // std::cout << msg->lighthouse << "-" << static_cast<int>(msg->axis) << " ";
  for (std::vector<hive::ViveLightSample>::const_iterator li_it = msg->samples.begin();
    li_it != msg->samples.end(); li_it++) {
    // Detect non-sense data
    if (li_it->sensor == -1
      || li_it->angle > M_PI/3
      || li_it->angle < -M_PI/3) {
      continue;
    }

    // New Light
    Light light;
    light.sensor_id = li_it->sensor;
    light.angle = li_it->angle;
    light.timecode = li_it->timecode;
    light.length = li_it->length;

    // std::cout << li_it->sensor << ":" << li_it->angle << " ";

    // Add data to the axis
    observations_[msg->lighthouse].axis[msg->axis].lights.push_back(light);
    observations_[msg->lighthouse].axis[msg->axis].stamp = msg->header.stamp;
  }
  // std::cout << std::endl;

  // Remove old data
  for (LightData::iterator lh_it = observations_.begin();
    lh_it != observations_.end(); lh_it++) {
    for (std::map<uint8_t, LightVecStamped>::iterator ax_it = lh_it->second.axis.begin();
      ax_it != lh_it->second.axis.end(); ax_it++) {
      ros::Duration elapsed = ax_it->second.stamp - msg->header.stamp;
      if (elapsed.toNSec() >= 50e6)
        ax_it->second.lights.clear();
    }
  }

  // Solve if we have data
  ros::Duration elapsed = observations_[msg->lighthouse].axis[VERTICAL].stamp -
    observations_[msg->lighthouse].axis[HORIZONTAL].stamp;
  if (observations_[msg->lighthouse].axis[HORIZONTAL].lights.size() > 3
    && observations_[msg->lighthouse].axis[VERTICAL].lights.size() > 3) {
    std::thread poseSolver(&ComputeTransformBundle,
      observations_,
      &tracker_pose_,
      &extrinsics_,
      &environment_,
      solveMutex_,
      &lh_extrinsics_);
    poseSolver.detach();
  }
  return;
}

bool ViveSolve::GetTransform(geometry_msgs::TransformStamped &msg) {
  solveMutex_->lock();
  // Filling the translation data
  if (!tracker_pose_.valid) {
    solveMutex_->unlock();

    return false;
  }
  msg.transform.translation.x = tracker_pose_.transform[0];
  msg.transform.translation.y = tracker_pose_.transform[1];
  msg.transform.translation.z = tracker_pose_.transform[2];
  // Axis Angle convertion to quaternion
  Eigen::Vector3d axis_angle_vec = Eigen::Vector3d(
    tracker_pose_.transform[3],
    tracker_pose_.transform[4],
    tracker_pose_.transform[5]);
  double angle = axis_angle_vec.norm();
  axis_angle_vec.normalize();
  Eigen::AngleAxisd axis_angle = Eigen::AngleAxisd(angle, axis_angle_vec);
  Eigen::Quaterniond quaternion_vec = Eigen::Quaterniond(axis_angle);
  // Filling the orietation data
  msg.transform.rotation.x = quaternion_vec.x();
  msg.transform.rotation.y = quaternion_vec.y();
  msg.transform.rotation.z = quaternion_vec.z();
  msg.transform.rotation.w = quaternion_vec.w();
  // Setting the frames
  msg.child_frame_id = tracker_.serial;
  msg.header.frame_id = environment_.vive.child_frame;
  msg.header.stamp = ros::Time::now();
  // Prevent repeated use of the same pose
  tracker_pose_.valid = false;
  solveMutex_->unlock();
  return true;
}

bool ViveSolve::Initialize(Tracker const& tracker) {
  for (std::map<uint8_t, Sensor>::const_iterator sn_it = tracker.sensors.begin();
    sn_it != tracker.sensors.end(); sn_it++) {
    if (unsigned(sn_it->first) >= TRACKER_SENSORS_NUMBER
      || unsigned(sn_it->first) < 0) {
      ROS_FATAL("Sensor ID is invalid.");
      return false;
    }
    extrinsics_.positions[3 * unsigned(sn_it->first)]     = sn_it->second.position.x;
    extrinsics_.positions[3 * unsigned(sn_it->first) + 1] = sn_it->second.position.y;
    extrinsics_.positions[3 * unsigned(sn_it->first) + 2] = sn_it->second.position.z;
    extrinsics_.normals[3 * unsigned(sn_it->first)]       = sn_it->second.normal.x;
    extrinsics_.normals[3 * unsigned(sn_it->first) + 1]   = sn_it->second.normal.y;
    extrinsics_.normals[3 * unsigned(sn_it->first) + 2]   = sn_it->second.normal.z;
  }
  extrinsics_.size = tracker.sensors.size();
  extrinsics_.radius = 0.005;
  tracker_ = tracker;
  return true;
}

bool ViveSolve::Initialize(Environment const& environment,
  Tracker const& tracker) {
  for (std::map<uint8_t, Sensor>::const_iterator sn_it = tracker.sensors.begin();
    sn_it != tracker.sensors.end(); sn_it++) {
    if (unsigned(sn_it->first) >= TRACKER_SENSORS_NUMBER
      || unsigned(sn_it->first) < 0) {
      ROS_FATAL("Sensor ID is invalid.");
      return false;
    }
    extrinsics_.positions[3 * unsigned(sn_it->first)]     = sn_it->second.position.x;
    extrinsics_.positions[3 * unsigned(sn_it->first) + 1] = sn_it->second.position.y;
    extrinsics_.positions[3 * unsigned(sn_it->first) + 2] = sn_it->second.position.z;
    extrinsics_.normals[3 * unsigned(sn_it->first)]       = sn_it->second.normal.x;
    extrinsics_.normals[3 * unsigned(sn_it->first) + 1]   = sn_it->second.normal.y;
    extrinsics_.normals[3 * unsigned(sn_it->first) + 2]   = sn_it->second.normal.z;
  }
  extrinsics_.size = tracker.sensors.size();
  extrinsics_.radius = 0.005;
  tracker_ = tracker;
  environment_ = environment;
  for (size_t i = 0; i < 6; i++) tracker_pose_.transform[i] = solve::start_pose[i];
  return true;
}

bool ViveSolve::Update(Environment const& environment) {
  environment_ = environment;
  return true;
}

bool ViveSolve::Update(std::map<std::string, Lighthouse> const& lh_extrinsics) {
  lh_extrinsics_ = lh_extrinsics;
  return true;
}

struct PoseHorizontalCost {
explicit PoseHorizontalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    data_ = data;
    correction_ = correction;
    tracker_ = tracker;
    lighthouse_ = lighthouse;
  }

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const {
    Eigen::Matrix<T, 3, 1> lPt;
    lPt << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> lRt;
    ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }


      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
};

struct PoseVerticalCost {
explicit PoseVerticalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    data_ = data;
    correction_ = correction;
    tracker_ = tracker;
    lighthouse_ = lighthouse;
  }

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const {
    Eigen::Matrix<T, 3, 1> lPt;
    lPt << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> lRt;
    ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }

      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
};

bool ViveSolve::SolvePose(hive::ViveLight & horizontal_observations,
  hive::ViveLight & vertical_observations,
  geometry_msgs::TransformStamped & tf,
  Tracker & tracker,
  Lighthouse & lighthouse,
  bool correction) {
  // Initializations
  double pose[6];

  ceres::Problem problem;

  // Data conversion
  pose[0] = tf.transform.translation.x;
  pose[1] = tf.transform.translation.y;
  pose[2] = tf.transform.translation.z;
  Eigen::Quaterniond Q(tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);
  Eigen::AngleAxisd AA(Q);
  pose[3] = AA.axis()(0) * AA.angle();
  pose[4] = AA.axis()(1) * AA.angle();
  pose[5] = AA.axis()(2) * AA.angle();

  ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4> * hcost =
    new ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4>
    (new PoseHorizontalCost(horizontal_observations, tracker, lighthouse.horizontal_motor,
      correction));
  hcost->AddParameterBlock(6);
  hcost->SetNumResiduals(horizontal_observations.samples.size());
  problem.AddResidualBlock(hcost,
    NULL,
    pose);

  ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4> * vcost =
    new ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4>
    (new PoseVerticalCost(vertical_observations, tracker, lighthouse.vertical_motor,
      correction));
  vcost->AddParameterBlock(6);
  vcost->SetNumResiduals(vertical_observations.samples.size());
  problem.AddResidualBlock(vcost,
    NULL,
    pose);

  // Check data here
  // // Sensors
  // std::cout << tracker.serial << std::endl;
  // for (auto sn_it = tracker.sensors.begin();
  //   sn_it != tracker.sensors.end(); sn_it++) {
  //   std::cout << (int)sn_it->first << " "
  //     << sn_it->second.position.x << ", "
  //     << sn_it->second.position.y << ", "
  //     << sn_it->second.position.z << std::endl;
  // }
  // std::cout << std::endl;

  // // Horizontal observations
  // std::cout << horizontal_observations.lighthouse << " "
  //   << (int)horizontal_observations.axis << std::endl;
  // for (auto li_it = horizontal_observations.samples.begin();
  //   li_it != horizontal_observations.samples.end(); li_it++) {
  //   std::cout << li_it->sensor << " " << li_it->angle << std::endl;
  // }
  // std::cout << std::endl;

  // // Vertical observations
  // std::cout << vertical_observations.lighthouse << " "
  //   << (int)vertical_observations.axis << std::endl;
  // for (auto li_it = vertical_observations.samples.begin();
  //   li_it != vertical_observations.samples.end(); li_it++) {
  //   std::cout << li_it->sensor << " " << li_it->angle << std::endl;
  // }
  // std::cout << std::endl;

  // // Correction
  // std::cout << lighthouse.serial << std::endl;
  // std::cout << "Horizontal Motor "
  //   << lighthouse.horizontal_motor.phase << ", "
  //   << lighthouse.horizontal_motor.tilt << ", "
  //   << lighthouse.horizontal_motor.gib_phase << ", "
  //   << lighthouse.horizontal_motor.gib_magnitude << ", "
  //   << lighthouse.horizontal_motor.curve << std::endl;
  // std::cout << "Vertical Motor "
  //   << lighthouse.vertical_motor.phase << ", "
  //   << lighthouse.vertical_motor.tilt << ", "
  //   << lighthouse.vertical_motor.gib_phase << ", "
  //   << lighthouse.vertical_motor.gib_magnitude << ", "
  //   << lighthouse.vertical_motor.curve << std::endl;

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 1.0;

  ceres::Solve(options, &problem, &summary);

  // Obtain the angles again and compare the results
  {
    std::cout << "CT: " << summary.final_cost << ", " << summary.num_residual_blocks << ", " << summary.num_residuals << " - "
      << pose[0] << ", "
      << pose[1] << ", "
      << pose[2] << ", "
      << pose[3] << ", "
      << pose[4] << ", "
      << pose[5];
    if (correction) {
      std::cout << " - CORRECTED" << std::endl;
    } else {
      std::cout << " - NOT CORRECTED" << std::endl;
    }
  }

  // Check if valid
  double pose_norm = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
  if (summary.final_cost > 1e-5* 0.5 * static_cast<double>(unsigned( vertical_observations.samples.size()
    + horizontal_observations.samples.size()))
    || pose_norm > 10
    || pose[2] <= 0 ) {
    return false;
  }

  // Change the axis angle to an acceptable interval
  double angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  while (angle_norm > M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm - 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm - 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm - 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }
  while (angle_norm < - M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm + 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm + 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm + 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }

  // Save the solved pose
  tf.transform.translation.x = pose[0];
  tf.transform.translation.y = pose[1];
  tf.transform.translation.z = pose[2];

  Eigen::Vector3d A(pose[3], pose[4], pose[5]);
  AA = Eigen::AngleAxisd(A.norm(), A.normalized());
  Q = Eigen::Quaterniond(AA);
  tf.transform.rotation.w = Q.w();
  tf.transform.rotation.x = Q.x();
  tf.transform.rotation.y = Q.y();
  tf.transform.rotation.z = Q.z();

  tf.header.frame_id = lighthouse.serial;
  tf.child_frame_id = tracker.serial;

  return true;
}

struct RayVerticalAngle{
  explicit RayVerticalAngle(LightVec vertical_observations, bool correction) :
  vertical_observations_(vertical_observations), correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    /*Rotation matrix from the tracker's frame to the lighthouse's frame*/
    Eigen::Matrix<T, 3, 3> lRt;
    /*Position of the tracker in the lighthouse's frame*/
    Eigen::Matrix<T, 3, 1> lPt;
    /*Position of the sensor in the tracker's frame*/
    Eigen::Matrix<T, 3, 1> tPs;

    lPt << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];

    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], lRt.data());
    // std::cout << "INSIDE VERTICAL FUNCTOR" << std::endl;
    for (size_t i = 0; i < vertical_observations_.size(); i++) {
      tPs << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS][PHASE];
      T tilt = parameters[LH_EXTRINSICS][TILT];
      T gib_phase = parameters[LH_EXTRINSICS][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS][CURVE];


      if (correction_) {
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }
      // std::cout << "V " << vertical_observations_[i].sensor_id << " - "
      // << lPs(0) << ", " << lPs(1) << ", " << lPs(2)
      // << " - " << ang << " " << vertical_observations_[i].angle << std::endl;
      residual[i] = T(vertical_observations_[i].angle) - ang;
      // std::cout << vertical_observations_[i].sensor_id << ": "
      //   << ang  << " - "
      //   << vertical_observations_[i].angle << " - "
      //   << lPt(0) << ", "
      //   << lPt(1) << ", "
      //   << lPt(2) << std::endl;
    }

    return true;
  }

 private:
  LightVec vertical_observations_;
  bool correction_;
};

struct RayHorizontalAngle{
  explicit RayHorizontalAngle(LightVec horizontal_observations, bool correction) :
  horizontal_observations_(horizontal_observations), correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    /*Rotation matrix from the tracker's frame to the lighthouse's frame*/
    Eigen::Matrix<T, 3, 3> lRt;
    /*Position of the tracker in the lighthouse's frame*/
    Eigen::Matrix<T, 3, 1> lPt;
    /*Position of the sensor in the tracker's frame*/
    Eigen::Matrix<T, 3, 1> tPs;

    lPt << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];

    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], lRt.data());

    // std::cout << "INSIDE HORIZONTAL FUNCTOR" << std::endl;
    for (size_t i = 0; i < horizontal_observations_.size(); i++) {
      tPs << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS][PHASE];
      T tilt = parameters[LH_EXTRINSICS][TILT];
      T gib_phase = parameters[LH_EXTRINSICS][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS][CURVE];


      if (correction_) {
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }
      // std::cout << "H " << horizontal_observations_[i].sensor_id << " - "
      // << lPs(0) << ", " << lPs(1) << ", " << lPs(2)
      // << " - " << ang << " " << horizontal_observations_[i].angle << std::endl;
      residual[i] = T(horizontal_observations_[i].angle) - ang;
      // std::cout << horizontal_observations_[i].sensor_id << ": "
      //   << ang  << " - "
      //   << horizontal_observations_[i].angle << " - "
      //   << lPt(0) << ", "
      //   << lPt(1) << ", "
      //   << lPt(2) << std::endl;
    }

    return true;
  }

 private:
  LightVec horizontal_observations_;
  bool correction_;
};

bool ComputeTransform(AxisLightVec observations,
  SolvedPose * pose_tracker,
  std::string * last_lh_pose,
  Extrinsics * extrinsics,
  std::mutex * solveMutex,
  Lighthouse * lh_extrinsics) {
  solveMutex->lock();
  ceres::Problem problem;
  double pose[6], lighthouseZeros[6];
  for (int i = 0; i < 6; i++) {
    pose[i] = pose_tracker->transform[i];
    lighthouseZeros[i] = 0.0;
  }
  solveMutex->unlock();

  bool correction = false;
  double lh_horizontal_extrinsics[5] = {0.0,0.0,0.0,0.0,0.0};
  double lh_vertical_extrinsics[5] = {0.0,0.0,0.0,0.0,0.0};
  if (lh_extrinsics != NULL && CORRECTION) {
    correction = true;
  } else {
    // pass
  }
  // correction = false;

  if (correction) {
    // Horizontal correction parameters
    lh_horizontal_extrinsics[0] = lh_extrinsics->horizontal_motor.phase;
    lh_horizontal_extrinsics[1] = lh_extrinsics->horizontal_motor.tilt;
    lh_horizontal_extrinsics[2] = lh_extrinsics->horizontal_motor.gib_phase;
    lh_horizontal_extrinsics[3] = lh_extrinsics->horizontal_motor.gib_magnitude;
    lh_horizontal_extrinsics[4] = lh_extrinsics->horizontal_motor.curve;
    // Vertical correction parameters
    lh_vertical_extrinsics[0] = lh_extrinsics->vertical_motor.phase;
    lh_vertical_extrinsics[1] = lh_extrinsics->vertical_motor.tilt;
    lh_vertical_extrinsics[2] = lh_extrinsics->vertical_motor.gib_phase;
    lh_vertical_extrinsics[3] = lh_extrinsics->vertical_motor.gib_magnitude;
    lh_vertical_extrinsics[4] = lh_extrinsics->vertical_motor.curve;
  }


  ceres::DynamicAutoDiffCostFunction<RayVerticalAngle, 4> * vertical_cost =
    new ceres::DynamicAutoDiffCostFunction<RayVerticalAngle, 4>
    (new RayVerticalAngle(observations.axis[VERTICAL].lights, correction));
  vertical_cost->AddParameterBlock(6);
  vertical_cost->AddParameterBlock(3 * extrinsics->size);
  vertical_cost->AddParameterBlock(6);
  vertical_cost->AddParameterBlock(1);
  vertical_cost->AddParameterBlock(5);
  vertical_cost->SetNumResiduals(observations.axis[VERTICAL].lights.size());

  ceres::ResidualBlockId vrb = problem.AddResidualBlock(vertical_cost,
    NULL,
    pose,
    extrinsics->positions,
    lighthouseZeros,
    &(extrinsics->radius),
    lh_vertical_extrinsics);
  std::vector<ceres::ResidualBlockId> v_residual_block_ids;
  v_residual_block_ids.push_back(vrb);

  ceres::DynamicAutoDiffCostFunction<RayHorizontalAngle, 4> * horizontal_cost =
    new ceres::DynamicAutoDiffCostFunction<RayHorizontalAngle, 4>
    (new RayHorizontalAngle(observations.axis[HORIZONTAL].lights, correction));
  horizontal_cost->AddParameterBlock(6);
  horizontal_cost->AddParameterBlock(3 * extrinsics->size);
  horizontal_cost->AddParameterBlock(6);
  horizontal_cost->AddParameterBlock(1);
  horizontal_cost->AddParameterBlock(5);
  horizontal_cost->SetNumResiduals(observations.axis[HORIZONTAL].lights.size());


  ceres::ResidualBlockId hrb = problem.AddResidualBlock(horizontal_cost,
    NULL,
    pose,
    extrinsics->positions,
    lighthouseZeros,
    &(extrinsics->radius),
    lh_horizontal_extrinsics);
  std::vector<ceres::ResidualBlockId> h_residual_block_ids;
  h_residual_block_ids.push_back(hrb);

  problem.SetParameterBlockConstant(extrinsics->positions);
  problem.SetParameterBlockConstant(lighthouseZeros);
  problem.SetParameterBlockConstant(&(extrinsics->radius));
  problem.SetParameterBlockConstant(lh_horizontal_extrinsics);
  problem.SetParameterBlockConstant(lh_vertical_extrinsics);

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 1.0;

  ceres::Solve(options, &problem, &summary);

  // ceres::Problem::EvaluateOptions pe_options;
  // double total_cost = 0.0;
  // std::vector<double> residuals;
  // pe_options.residual_blocks = h_residual_block_ids;
  // problem.Evaluate(pe_options, &total_cost, &residuals, nullptr, nullptr);
  // std::cout << "Horizontal Residuals" << std::endl;
  // for (size_t i = 0; i < residuals.size(); i++) {
  //   std::cout << i << ": " << residuals[i] << std::endl;
  // }
  // pe_options.residual_blocks = v_residual_block_ids;
  // problem.Evaluate(pe_options, &total_cost, &residuals, nullptr, nullptr);
  // std::cout << "Vertical Residuals" << std::endl;
  // for (size_t i = 0; i < residuals.size(); i++) {
  //   std::cout << i << ": " << residuals[i] << std::endl;
  // }

  // Obtain the angles again and compare the results
  {
  //   Eigen::Vector3d lPt = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  //   Eigen::Vector3d AA = Eigen::Vector3d(pose[3], pose[4], pose[5]);
  //   Eigen::Matrix3d lRt = Eigen::AngleAxisd(AA.norm(), AA / AA.norm()).toRotationMatrix();
  //   std::cout << "HORIZONTAL" << std::endl;
  //   double h_cost = 0;
  //   for (auto li_it = observations.axis[HORIZONTAL].lights.begin();
  //     li_it < observations.axis[HORIZONTAL].lights.end(); li_it++) {
  //     Eigen::Vector3d tPs = Eigen::Vector3d(extrinsics->positions[3*li_it->sensor_id+0],
  //       extrinsics->positions[3*li_it->sensor_id+1],
  //       extrinsics->positions[3*li_it->sensor_id+2]);
  //     Eigen::Vector3d lPs = lRt * tPs + lPt;
  //     double x = atan(lPs(0)/lPs(2)); // Horizontal angle
  //     double y = atan(lPs(1)/lPs(2)); // Vertical angle
  //     double phase = lh_horizontal_extrinsics[PHASE];
  //     double tilt = lh_horizontal_extrinsics[TILT];
  //     double gib_phase = lh_horizontal_extrinsics[GIB_PHASE];
  //     double gib_mag = lh_horizontal_extrinsics[GIB_MAG];
  //     double curve = lh_horizontal_extrinsics[CURVE];

  //     double ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      
  //     h_cost += pow(ang - li_it->angle, 2);
  //     std::cout << li_it->sensor_id << ": " << ang << " | " << li_it->angle << " - " << ang - li_it->angle << " | " << pow(ang - li_it->angle, 2) << std::endl;
  //   }
  //   double v_cost = 0;
  //   std::cout << "VERTICAL" << std::endl;
  //   for (auto li_it = observations.axis[VERTICAL].lights.begin();
  //     li_it < observations.axis[VERTICAL].lights.end(); li_it++) {
  //     Eigen::Vector3d tPs = Eigen::Vector3d(extrinsics->positions[3*li_it->sensor_id+0],
  //       extrinsics->positions[3*li_it->sensor_id+1],
  //       extrinsics->positions[3*li_it->sensor_id+2]);
  //     Eigen::Vector3d lPs = lRt * tPs + lPt;
  //     double x = atan(lPs(0)/lPs(2)); // Horizontal angle
  //     double y = atan(lPs(1)/lPs(2)); // Vertical angle
  //     double phase = lh_vertical_extrinsics[PHASE];
  //     double tilt = lh_vertical_extrinsics[TILT];
  //     double gib_phase = lh_vertical_extrinsics[GIB_PHASE];
  //     double gib_mag = lh_vertical_extrinsics[GIB_MAG];
  //     double curve = lh_vertical_extrinsics[CURVE];
      
  //     double ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      
  //     v_cost += pow(ang - li_it->angle, 2);
  //     std::cout << li_it->sensor_id << ": " << ang << " | " << li_it->angle << " - " << ang - li_it->angle << " | " << pow(ang - li_it->angle, 2) << std::endl;
  //   }
  //   std::cout << "HSUMMED " << h_cost << "   VSUMMED " << v_cost << std::endl;
  //   // std::cout << summary.FullReport() << std::endl;
  //   for (auto ax_it = observations.axis.begin();
  //     ax_it != observations.axis.end(); ax_it++) {
  //     std::cout << observations.lighthouse << "" << static_cast<int>(ax_it->first);
  //     for (auto li_it = ax_it->second.lights.begin();
  //       li_it != ax_it->second.lights.end(); li_it++) {
  //       std::cout << " " << li_it->sensor_id << ":" << li_it->angle;
  //     }
  //     std::cout << std::endl;
  //   }
  //   std::cout << "HM: " << lh_horizontal_extrinsics[0] << ", "
  //     << lh_horizontal_extrinsics[1] << ", "
  //     << lh_horizontal_extrinsics[2] << ", "
  //     << lh_horizontal_extrinsics[3] << ", "
  //     << lh_horizontal_extrinsics[4] << std::endl;
  //   std::cout << "VM: " << lh_vertical_extrinsics[0] << ", "
  //     << lh_vertical_extrinsics[1] << ", "
  //     << lh_vertical_extrinsics[2] << ", "
  //     << lh_vertical_extrinsics[3] << ", "
  //     << lh_vertical_extrinsics[4] << std::endl;
  //   for (size_t i = 0; i < extrinsics->size; i++) {
  //     std::cout << i << " - "
  //       << extrinsics->positions[3*i+0] << " "
  //       << extrinsics->positions[3*i+1] << " "
  //       << extrinsics->positions[3*i+2] << std::endl;
  //   }
    // solveMutex->unlock();
    std::cout << "CT: " << summary.final_cost << ", " << summary.num_residual_blocks << ", " << summary.num_residuals << " - "
      << pose[0] << ", "
      << pose[1] << ", "
      << pose[2] << ", "
      << pose[3] << ", "
      << pose[4] << ", "
      << pose[5];
    if (correction) {
      std::cout << " - CORRECTED" << std::endl;
    } else {
      std::cout << " - NOT CORRECTED" << std::endl;
    }


    // if (correction) {
    //   std::cout << "CORRECTION TRUE" << std::endl << std::endl;
    // } else {
    //   std::cout << "CORRECTION FALSE" << std::endl << std::endl;
    // }
  }

  // Check if valid
  double pose_norm = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
  if (summary.final_cost > 1e-5* 0.5 * static_cast<double>(unsigned( observations.axis[VERTICAL].lights.size()
    + observations.axis[HORIZONTAL].lights.size()))
    || pose_norm > 20
    || pose[2] <= 0 ) {
    return false;
  }

  double angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  // Change the axis angle to an acceptable interval
  while (angle_norm > M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm - 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm - 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm - 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }
  // Change the axis angle to an acceptable interval
  while (angle_norm < - M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm + 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm + 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm + 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }

  // Save the solved pose
  solveMutex->lock();
  for (int i = 0; i < 6; i++) {
    pose_tracker->transform[i] = pose[i];
  }
  pose_tracker->lighthouse = observations.lighthouse;
  pose_tracker->valid = true;
  // pose_tracker->stamp = ros::Time::now();
  *last_lh_pose = observations.lighthouse;
  solveMutex->unlock();

  return true;
}

struct BundleHorizontalAngle{
  explicit BundleHorizontalAngle(LightVec horizontal_observations, bool correction) :
  horizontal_observations_(horizontal_observations), correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    // Rotation matrix from the tracker's frame to the vive frame
    Eigen::Matrix<T, 3, 3> vRt;
    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], vRt.data());
    // Position of the tracker in the vive frame
    Eigen::Matrix<T, 3, 1> vPt;
    vPt << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];
    // Position of the sensor in the tracker's frame
    Eigen::Matrix<T, 3, 1> tPs;
    // Orientation of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[LIGHTHOUSE][3], vRl.data());
    Eigen::Matrix<T, 3, 3> lRv = vRl.transpose();
    // Position of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << parameters[LIGHTHOUSE][0], parameters[LIGHTHOUSE][1], parameters[LIGHTHOUSE][2];
    Eigen::Matrix<T, 3, 1> lPv = - lRv * vPl;

    // Create residuals
    for (size_t i = 0; i < horizontal_observations_.size(); i++) {
      // Position of the photodiode
      tPs << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRv * vRt * tPs + (lRv * vPt + lPv);
      // residual[i] = T(-horizontal_observations_[i].angle) - atan(lPs(1)/lPs(2));

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS][PHASE];
      T tilt = parameters[LH_EXTRINSICS][TILT];
      T gib_phase = parameters[LH_EXTRINSICS][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS][CURVE];
      if (correction_) {
        // Distortion correction
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }

      residual[i] = T(horizontal_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec horizontal_observations_;
  bool correction_;
};

struct BundleVerticalAngle{
  explicit BundleVerticalAngle(LightVec vertical_observations, bool correction) :
  vertical_observations_(vertical_observations), correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    // Rotation matrix from the tracker's frame to the vive frame
    Eigen::Matrix<T, 3, 3> vRt;
    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], vRt.data());
    // Position of the tracker in the vive frame
    Eigen::Matrix<T, 3, 1> vPt;
    vPt << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];
    // Position of the sensor in the tracker's frame
    Eigen::Matrix<T, 3, 1> tPs;
    // Orientation of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[LIGHTHOUSE][3], vRl.data());
    Eigen::Matrix<T, 3, 3> lRv = vRl.transpose();
    // Position of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << parameters[LIGHTHOUSE][0], parameters[LIGHTHOUSE][1], parameters[LIGHTHOUSE][2];
    Eigen::Matrix<T, 3, 1> lPv = - lRv * vPl;

    // Create residuals
    for (size_t i = 0; i < vertical_observations_.size(); i++) {
      // Position of the photodiode
      tPs << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRv * vRt * tPs + (lRv * vPt + lPv);
      // residual[i] = T(vertical_observations_[i].angle) - atan(lPs(0)/lPs(2));

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS][PHASE];
      T tilt = parameters[LH_EXTRINSICS][TILT];
      T gib_phase = parameters[LH_EXTRINSICS][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS][CURVE];
      if (correction_) {
        // Distortion correction
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }

      residual[i] = T(vertical_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec vertical_observations_;
  bool correction_;
};

bool ComputeTransformBundle(LightData observations,
  SolvedPose * pose_tracker,
  Extrinsics * extrinsics,
  Environment * environment,
  std::mutex * solveMutex,
  LighthouseMap * lighthouses) {
  solveMutex->lock();
  ceres::Problem problem;
  double pose[6];// = {0.017356, -0.00947887, 1.53151, -0.799895, 2.22509, -0.436057};
  std::map<std::string, double[6]> lighthouses_pose;
  std::map<std::string, double[5]> lh_vertical_extrinsics;
  std::map<std::string, double[5]> lh_horizontal_extrinsics;

  for (int i = 0; i < 6; i++) {
    pose[i] = pose_tracker->transform[i];
  }

  solveMutex->unlock();
  size_t n_sensors = 0, counter = 0;
  for (LightData::iterator ld_it = observations.begin();
    ld_it != observations.end(); ld_it++) {
    // Get lighthouse to angle axis
    bool lighthouse = false;
    bool correction = false;
    if (environment->lighthouses.find(ld_it->first) == environment->lighthouses.end())
      continue;

    if (lighthouses->find(ld_it->first) != lighthouses->end()
      && CORRECTION) {
      // std::cout << "CORRECTION " << ld_it->first << " TRUE" << std::endl;
      correction = true;
    } else {
      // std::cout << "CORRECTION " << ld_it->first << " FALSE" << std::endl;
    }
    // correction = false;

    Eigen::AngleAxisd tmp_AA = Eigen::AngleAxisd(
      Eigen::Quaterniond(
        environment->lighthouses[ld_it->first].rotation.w,
        environment->lighthouses[ld_it->first].rotation.x,
        environment->lighthouses[ld_it->first].rotation.y,
        environment->lighthouses[ld_it->first].rotation.z).toRotationMatrix());
    lighthouses_pose[ld_it->first][0] = environment->lighthouses[ld_it->first].translation.x;
    lighthouses_pose[ld_it->first][1] = environment->lighthouses[ld_it->first].translation.y;
    lighthouses_pose[ld_it->first][2] = environment->lighthouses[ld_it->first].translation.z;
    lighthouses_pose[ld_it->first][3] = tmp_AA.axis()(0) * tmp_AA.angle();
    lighthouses_pose[ld_it->first][4] = tmp_AA.axis()(1) * tmp_AA.angle();
    lighthouses_pose[ld_it->first][5] = tmp_AA.axis()(2) * tmp_AA.angle();
    // Filling the solve with the data
    if (ld_it->second.axis[HORIZONTAL].lights.size() != 0) {
      n_sensors += ld_it->second.axis[HORIZONTAL].lights.size();
      ceres::DynamicAutoDiffCostFunction<BundleHorizontalAngle, 4> * horizontal_cost =
      new ceres::DynamicAutoDiffCostFunction<BundleHorizontalAngle, 4>
      (new BundleHorizontalAngle(ld_it->second.axis[HORIZONTAL].lights, correction));
      horizontal_cost->AddParameterBlock(6);
      horizontal_cost->AddParameterBlock(3 * extrinsics->size);
      horizontal_cost->AddParameterBlock(6);
      horizontal_cost->AddParameterBlock(1);
      horizontal_cost->AddParameterBlock(5);
      horizontal_cost->SetNumResiduals(ld_it->second.axis[HORIZONTAL].lights.size());

      // Convert lh extrinsics to double*
      lh_horizontal_extrinsics[ld_it->first][0] = (*lighthouses)[ld_it->first].horizontal_motor.phase;
      lh_horizontal_extrinsics[ld_it->first][1] = (*lighthouses)[ld_it->first].horizontal_motor.tilt;
      lh_horizontal_extrinsics[ld_it->first][2] = (*lighthouses)[ld_it->first].horizontal_motor.gib_phase;
      lh_horizontal_extrinsics[ld_it->first][3] = (*lighthouses)[ld_it->first].horizontal_motor.gib_magnitude;
      lh_horizontal_extrinsics[ld_it->first][4] = (*lighthouses)[ld_it->first].horizontal_motor.curve;

      problem.AddResidualBlock(horizontal_cost,
        NULL,
        pose,
        extrinsics->positions,
        lighthouses_pose[ld_it->first],
        &(extrinsics->radius),
        lh_horizontal_extrinsics[ld_it->first]);
      // problem.AddResidualBlock(horizontal_cost,
      //   new ceres::CauchyLoss(0.5),
      //   pose,
      //   extrinsics->positions,
      //   lighthouses_pose[ld_it->first],
      //   &(extrinsics->radius),
      //   lh_horizontal_extrinsics[ld_it->first]);

      problem.SetParameterBlockConstant(lh_horizontal_extrinsics[ld_it->first]);
      lighthouse = lighthouse || true;
    }
    if (ld_it->second.axis[VERTICAL].lights.size() != 0) {
      n_sensors += ld_it->second.axis[VERTICAL].lights.size();
      ceres::DynamicAutoDiffCostFunction<BundleVerticalAngle, 4> * vertical_cost =
      new ceres::DynamicAutoDiffCostFunction<BundleVerticalAngle, 4>
      (new BundleVerticalAngle(ld_it->second.axis[VERTICAL].lights, correction));
      vertical_cost->AddParameterBlock(6);
      vertical_cost->AddParameterBlock(3 * extrinsics->size);
      vertical_cost->AddParameterBlock(6);
      vertical_cost->AddParameterBlock(1);
      vertical_cost->AddParameterBlock(5);
      vertical_cost->SetNumResiduals(ld_it->second.axis[VERTICAL].lights.size());

      // Convert lh extrinsics to double*
      lh_vertical_extrinsics[ld_it->first][0] = (*lighthouses)[ld_it->first].vertical_motor.phase;
      lh_vertical_extrinsics[ld_it->first][1] = (*lighthouses)[ld_it->first].vertical_motor.tilt;
      lh_vertical_extrinsics[ld_it->first][2] = (*lighthouses)[ld_it->first].vertical_motor.gib_phase;
      lh_vertical_extrinsics[ld_it->first][3] = (*lighthouses)[ld_it->first].vertical_motor.gib_magnitude;
      lh_vertical_extrinsics[ld_it->first][4] = (*lighthouses)[ld_it->first].vertical_motor.curve;

      problem.AddResidualBlock(vertical_cost,
        NULL,
        pose,
        extrinsics->positions,
        lighthouses_pose[ld_it->first],
        &(extrinsics->radius),
        lh_vertical_extrinsics[ld_it->first]);
      // problem.AddResidualBlock(vertical_cost,
      //   new ceres::CauchyLoss(0.5),
      //   pose,
      //   extrinsics->positions,
      //   lighthouses_pose[ld_it->first],
      //   &(extrinsics->radius),
      //   lh_vertical_extrinsics[ld_it->first]);

      problem.SetParameterBlockConstant(lh_vertical_extrinsics[ld_it->first]);
      lighthouse = lighthouse || true;
    }
    if (lighthouse) {
      problem.SetParameterBlockConstant(lighthouses_pose[ld_it->first]);
      counter++;
    }
  }
  // Exit in case no lighthouses are available
  if (counter == 0) {
    return false;
  }
  problem.SetParameterBlockConstant(extrinsics->positions);
  problem.SetParameterBlockConstant(&(extrinsics->radius));

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;

  ceres::Solve(options, &problem, &summary);

  solveMutex->lock();
  // for (LightData::iterator ld_it = observations.begin();
  //   ld_it != observations.end(); ld_it++) {
  //   for (auto ax_it = ld_it->second.axis.begin();
  //     ax_it != ld_it->second.axis.end(); ax_it++) {
  //     std::cout << ld_it->first << "-" << static_cast<int>(ax_it->first);
  //     for (auto li_it = ax_it->second.lights.begin();
  //       li_it != ax_it->second.lights.end(); li_it++) {
  //       std::cout << " " << li_it->sensor_id << ":" << li_it->angle;
  //     }
  //     std::cout << std::endl;
  //   }
  // }
  // for (auto lh_it = lighthouses_pose.begin(); lh_it != lighthouses_pose.end();
  //   lh_it++) {
  //   std::cout << lh_it->first << " - "
  //     << lh_it->second[0] << ", "
  //     << lh_it->second[1] << ", "
  //     << lh_it->second[2] << ", "
  //     << lh_it->second[3] << ", "
  //     << lh_it->second[4] << ", "
  //     << lh_it->second[5] << std::endl;
  //   std::cout << "HM: " << lh_horizontal_extrinsics[lh_it->first][0] << ", "
  //     << lh_horizontal_extrinsics[lh_it->first][1] << ", "
  //     << lh_horizontal_extrinsics[lh_it->first][2] << ", "
  //     << lh_horizontal_extrinsics[lh_it->first][3] << ", "
  //     << lh_horizontal_extrinsics[lh_it->first][4] << std::endl;
  //   std::cout << "VM: " << lh_vertical_extrinsics[lh_it->first][0] << ", "
  //     << lh_vertical_extrinsics[lh_it->first][1] << ", "
  //     << lh_vertical_extrinsics[lh_it->first][2] << ", "
  //     << lh_vertical_extrinsics[lh_it->first][3] << ", "
  //     << lh_vertical_extrinsics[lh_it->first][4] << std::endl;
  // }
  // for (size_t i = 0; i < extrinsics->size; i++) {
  //   std::cout << i << " - "
  //     << extrinsics->positions[3*i+0] << " "
  //     << extrinsics->positions[3*i+1] << " "
  //     << extrinsics->positions[3*i+2] << std::endl;
  // }
  std::cout << std::setprecision(4) << "CTB: " 
    << std::setprecision(4) << summary.final_cost << " - "
    << std::setprecision(4) << pose[0] << ", "
    << std::setprecision(4) << pose[1] << ", "
    << std::setprecision(4) << pose[2] << ", "
    << std::setprecision(4) << pose[3] << ", "
    << std::setprecision(4) << pose[4] << ", "
    << std::setprecision(4) << pose[5] << std::endl << std::endl;
  solveMutex->unlock();


  // Check if valid
  double pose_norm = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
  if (summary.final_cost > 1e-5* static_cast<double>(unsigned(n_sensors))
    || pose_norm > 20
    || pose[2] <= 0 ) {
    return false;
  }

  double angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  // Change the axis angle to an acceptable interval
  while (angle_norm > M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm - 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm - 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm - 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }
  // Change the axis angle to an acceptable interval
  while (angle_norm < - M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm + 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm + 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm + 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }

  // Save the solved pose
  solveMutex->lock();
  for (int i = 0; i < 6; i++) {
    pose_tracker->transform[i] = pose[i];
  }
  pose_tracker->valid = true;
  pose_tracker->stamp = ros::Time::now();
  solveMutex->unlock();

  return true;
}