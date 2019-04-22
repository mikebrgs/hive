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
#include <hive/vive_calibrate.h>

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

// Constructor just sets the callback function
ViveCalibrate::ViveCalibrate(CallbackFn cb) : cb_(cb), active_(false) {
  mutex_ = new std::mutex();
}

// Reset
bool ViveCalibrate::Reset() {
  if (!mutex_->try_lock()) return false;
  data_pair_map_.clear();
  mutex_->unlock();
  return true;
}

// Add an IMU measurement
bool ViveCalibrate::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (!mutex_->try_lock()) return false;
  data_pair_map_[msg->header.frame_id].second.push_back(*msg);
  mutex_->unlock();
  return true;
}

// Add a light measurement
bool ViveCalibrate::AddLight(const hive::ViveLight::ConstPtr& msg) {
  if (!mutex_->try_lock()) return false;

  if (msg == NULL) {
    return false;
  }

  Sweep sweep;
  // Iterate sensors
  for (std::vector<hive::ViveLightSample>::const_iterator li_it = msg->samples.begin();
    li_it != msg->samples.end(); li_it++) {
    // Check the data
    if (li_it->sensor == -1
      || li_it->angle < -M_PI/3
      || li_it->angle > M_PI/3) {
      continue;
    }
    // new data structure
    Light light;
    light.sensor_id = li_it->sensor;
    light.angle = li_it->angle;
    light.timecode = li_it->timecode;
    light.length = li_it->length;
    sweep.lights.push_back(light);
  }
  sweep.lighthouse = msg->lighthouse;
  sweep.axis = msg->axis;

  // std::cout << sweep.lighthouse << " " << sweep.axis << " " << msg->header.frame_id << std::endl;

  // Store the data
  if (msg->samples.size() > 0) {
    data_pair_map_[msg->header.frame_id].first.push_back(sweep);
  }

  mutex_->unlock();
  return true;
}

bool ViveCalibrate::Initialize(Calibration & calibration) {
  if (!mutex_->try_lock()) return false;
  calibration_ = calibration;
  mutex_->unlock();
  return true;
}

bool ViveCalibrate::Update(LighthouseMap  const& lh_extrinsics) {
  calibration_.lighthouses = lh_extrinsics;
  return true;
}


void ViveCalibrate::PrintStuff(){
  for (auto tr_it = calibration_.trackers.begin(); tr_it != calibration_.trackers.end(); tr_it++) {
    // std::cout << "SENSOR SIZE " << tr_it->second.sensors.size() << std::endl;
  }
  return;
}

// Start solving in a parallel thread
bool ViveCalibrate::Solve() {
  // Notify any previous solution
  if (active_) return false;
  active_ = true;

  // Wait for thread to join, in case of old solution
  std::thread thread = std::thread(ViveCalibrate::WorkerThread,
    cb_,
    mutex_,
    data_pair_map_,
    calibration_);
  thread.join();

  active_ = false;
  return true;
}

struct CalibHorizontalAngle{
  explicit CalibHorizontalAngle(LightVec horizontal_observations, PoseVM world_tracker_transforms, bool correction) :
  horizontal_observations_(horizontal_observations),
  world_tracker_transforms_(world_tracker_transforms),
  correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    // Rotation matrix from the tracker's frame to the lighthouse's frame
    Eigen::Matrix<T, 3, 3> lRw;
    // Position of the tracker in the lighthouse's frame
    Eigen::Matrix<T, 3, 1> lPw;
    // Position of the sensor in the tracker's frame
    Eigen::Matrix<T, 3, 1> tPs;
    // Position of the tracker in the world frame
    Eigen::Matrix<T, 3, 1> wPt;
    // Rotation of the tracker in the world frame
    Eigen::Matrix<T, 3, 3> wRt;
    // std::cout << "H1" << std::endl;

    // Filling with data
    lPw << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];
    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], lRw.data());
    wPt(0) = T(world_tracker_transforms_.first(0));
    wPt(1) = T(world_tracker_transforms_.first(1));
    wPt(2) = T(world_tracker_transforms_.first(2));
    wRt(0, 0) = T(world_tracker_transforms_.second(0, 0));
    wRt(0, 1) = T(world_tracker_transforms_.second(0, 1));
    wRt(0, 2) = T(world_tracker_transforms_.second(0, 2));
    wRt(1, 0) = T(world_tracker_transforms_.second(1, 0));
    wRt(1, 1) = T(world_tracker_transforms_.second(1, 1));
    wRt(1, 2) = T(world_tracker_transforms_.second(1, 2));
    wRt(2, 0) = T(world_tracker_transforms_.second(2, 0));
    wRt(2, 1) = T(world_tracker_transforms_.second(2, 1));
    wRt(2, 2) = T(world_tracker_transforms_.second(2, 2));
    // std::cout << "H2" << std::endl;

    for (size_t i = 0; i < horizontal_observations_.size(); i++) {
      tPs << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRw * (wRt * tPs + wPt) + lPw;
      // std::cout << "wRt: "
      //   << wRt(0,0) << ", "
      //   << wRt(0,1) << ", "
      //   << wRt(0,2) << ", "
      //   << wRt(1,0) << ", "
      //   << wRt(1,1) << ", "
      //   << wRt(1,2) << ", "
      //   << wRt(2,0) << ", "
      //   << wRt(2,1) << ", "
      //   << wRt(2,2) << std::endl;
      // std::cout << "lPs: "
      //   << lPs(0) << ", "
      //   << lPs(1) << ", "
      //   << lPs(2) << std::endl;
      // residual[i] = T(-horizontal_observations_[i].angle) - atan(lPs(1)/lPs(2));
      // std::cout << horizontal_observations_[i].sensor_id << " - "
      //   << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 0] << ", "
      //   << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 1] << ", "
      //   << parameters[EXTRINSICS][3*horizontal_observations_[i].sensor_id + 2] << std::endl;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS-1][PHASE];
      T tilt = parameters[LH_EXTRINSICS-1][TILT];
      T gib_phase = parameters[LH_EXTRINSICS-1][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS-1][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS-1][CURVE];
      // std::cout << "HExt "
      //   << parameters[LH_EXTRINSICS-1][PHASE] << ", "
      //   << parameters[LH_EXTRINSICS-1][TILT] << ", "
      //   << parameters[LH_EXTRINSICS-1][GIB_PHASE] << ", "
      //   << parameters[LH_EXTRINSICS-1][GIB_MAG] << ", "
      //   << parameters[LH_EXTRINSICS-1][CURVE] << std::endl;
      if (correction_) {
        // Distortion correction
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }
      // std::cout << "HOR " << ang << " - " << horizontal_observations_[i].angle << std::endl;

      residual[i] = T(horizontal_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec horizontal_observations_;
  PoseVM world_tracker_transforms_;
  bool correction_;
};

struct CalibVerticalAngle{
  explicit CalibVerticalAngle(LightVec vertical_observations, PoseVM world_tracker_transforms, bool correction) :
  vertical_observations_(vertical_observations),
  world_tracker_transforms_(world_tracker_transforms),
  correction_(correction) {}

  template <typename T> bool operator()(const T* const * parameters, T * residual) const {
    // Rotation matrix from the world's frame to the lighthouse's frame
    Eigen::Matrix<T, 3, 3> lRw;
    // Position of the world in the lighthouse's frame
    Eigen::Matrix<T, 3, 1> lPw;
    // Position of the sensor in the tracker's frame
    Eigen::Matrix<T, 3, 1> tPs;
    // Position of the tracker in the world frame
    Eigen::Matrix<T, 3, 1> wPt;
    // Rotation of the tracker in the world frame
    Eigen::Matrix<T, 3, 3> wRt;
    // std::cout << "H1" << std::endl;
    // Filling with data
    lPw << parameters[POSE][0], parameters[POSE][1], parameters[POSE][2];
    ceres::AngleAxisToRotationMatrix(&parameters[POSE][3], lRw.data());
    wPt(0) = T(world_tracker_transforms_.first(0));
    wPt(1) = T(world_tracker_transforms_.first(1));
    wPt(2) = T(world_tracker_transforms_.first(2));
    wRt(0, 0) = T(world_tracker_transforms_.second(0, 0));
    wRt(0, 1) = T(world_tracker_transforms_.second(0, 1));
    wRt(0, 2) = T(world_tracker_transforms_.second(0, 2));
    wRt(1, 0) = T(world_tracker_transforms_.second(1, 0));
    wRt(1, 1) = T(world_tracker_transforms_.second(1, 1));
    wRt(1, 2) = T(world_tracker_transforms_.second(1, 2));
    wRt(2, 0) = T(world_tracker_transforms_.second(2, 0));
    wRt(2, 1) = T(world_tracker_transforms_.second(2, 1));
    wRt(2, 2) = T(world_tracker_transforms_.second(2, 2));
    // std::cout << "H2" << std::endl;

    for (size_t i = 0; i < vertical_observations_.size(); i++) {
      tPs << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 0],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 1],
      parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 2];
      Eigen::Matrix<T, 3, 1> lPs = lRw * (wRt * tPs + wPt) + lPw;
      // std::cout << "wRt: "
      //   << wRt(0,0) << ", "
      //   << wRt(0,1) << ", "
      //   << wRt(0,2) << ", "
      //   << wRt(1,0) << ", "
      //   << wRt(1,1) << ", "
      //   << wRt(1,2) << ", "
      //   << wRt(2,0) << ", "
      //   << wRt(2,1) << ", "
      //   << wRt(2,2) << std::endl;
      // std::cout << "lPs: "
      //   << lPs(0) << ", "
      //   << lPs(1) << ", "
      //   << lPs(2) << std::endl;

      // std::cout << vertical_observations_[i].sensor_id << " - "
      //   << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 0] << ", "
      //   << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 1] << ", "
      //   << parameters[EXTRINSICS][3*vertical_observations_[i].sensor_id + 2] << std::endl;
      // residual[i] = T(vertical_observations_[i].angle) - atan(lPs(0)/lPs(2));
    // std::cout << "H3" << std::endl;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = parameters[LH_EXTRINSICS-1][PHASE];
      T tilt = parameters[LH_EXTRINSICS-1][TILT];
      T gib_phase = parameters[LH_EXTRINSICS-1][GIB_PHASE];
      T gib_mag = parameters[LH_EXTRINSICS-1][GIB_MAG];
      T curve = parameters[LH_EXTRINSICS-1][CURVE];
      // std::cout << "VExt "
      //   << parameters[LH_EXTRINSICS-1][PHASE] << ", "
      //   << parameters[LH_EXTRINSICS-1][TILT] << ", "
      //   << parameters[LH_EXTRINSICS-1][GIB_PHASE] << ", "
      //   << parameters[LH_EXTRINSICS-1][GIB_MAG] << ", "
      //   << parameters[LH_EXTRINSICS-1][CURVE] << std::endl;
      if (correction_) {
        // Distortion correction
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }
      // std::cout << "VERT " << ang << " - " << vertical_observations_[i].angle << std::endl;

      residual[i] = T(vertical_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec vertical_observations_;
  PoseVM world_tracker_transforms_;
  bool correction_;
};

// Takes the calibration data and returns the poses of the
// trackers in the world frame
bool GetBodyTransformsInW(PoseTrackers * body_transforms,
  std::string * calibration_body,
  Calibration calibration) {
  PoseVM body_offset;
  Eigen::Quaterniond Quat;
  if (calibration.environment.bodies.size() == 0) {
    ROS_FATAL("Please set body frame for calibration");
    return false;
  }
  body_offset.first = Eigen::Vector3d(calibration.environment.offset.translation.x,
    calibration.environment.offset.translation.y,
    calibration.environment.offset.translation.z);
  Quat = Eigen::Quaterniond(calibration.environment.offset.rotation.w,
    calibration.environment.offset.rotation.x,
    calibration.environment.offset.rotation.y,
    calibration.environment.offset.rotation.z);
  // std::cout << "Q " << aux_q.w()
  //   << ", " << aux_q.x()
  //   << ", " << aux_q.y()
  //   << ", " << aux_q.z()<< std::endl;
  Quat.normalize();
  body_offset.second = Quat.toRotationMatrix();

  // std::cout << "wTb: " << " \nP = " <<
  //   body_offset.first << " \nR = " <<
  //   body_offset.second << "" << std::endl;

  *calibration_body = calibration.environment.bodies.begin()->second.child_frame;
  for (std::map<std::string, Transform>::iterator bd_it = calibration.environment.bodies.begin();
    bd_it != calibration.environment.bodies.end(); bd_it++) {
    // The first body on the vector for calibration
    if (bd_it->second.child_frame != *calibration_body)
      continue;
    // Saving the transforms of the body frame in the tracker frame
    (*body_transforms)[bd_it->second.parent_frame].first =
      Eigen::Vector3d(bd_it->second.translation.x,
      bd_it->second.translation.y,
      bd_it->second.translation.z);
    Quat = Eigen::Quaterniond(bd_it->second.rotation.w,
      bd_it->second.rotation.x,
      bd_it->second.rotation.y,
      bd_it->second.rotation.z);
    Quat.normalize();
    (*body_transforms)[bd_it->second.parent_frame].second =
      Quat.toRotationMatrix();

  // std::cout << "tTb: " << calibration_body << " \nP = " <<
  //   (*body_transforms)[bd_it->second.parent_frame].first << " \nR = " <<
  //   (*body_transforms)[bd_it->second.parent_frame].second << "" << std::endl;

    // Saving the transforms of the tracker frame in the body frame
    Eigen::Vector3d tmP;
    Eigen::Matrix3d tmR;
    tmR = (*body_transforms)[bd_it->second.parent_frame].second.transpose();
    (*body_transforms)[bd_it->second.parent_frame].second = body_offset.second * tmR;
    tmP = - tmR * (*body_transforms)[bd_it->second.parent_frame].first;
    (*body_transforms)[bd_it->second.parent_frame].first = body_offset.second * tmP + body_offset.first;
    // std::cout << "wTt: " << calibration_body << " " << bd_it->first << " \nP = " <<
    //   (*body_transforms)[bd_it->second.parent_frame].first << " \nR = " <<
    //   (*body_transforms)[bd_it->second.parent_frame].second << "" << std::endl;
  }
  return true;
}

// Takes the observation data and returns the poses of the lighthouses
// in the trackers' frames
bool GetLhTransformsInTr(PoseMap * poses,
  DataPairMap data_pair_map,
  PoseTrackers body_transforms,
  Calibration calibration,
  std::string calibration_body) {
  PosesMapMap poses_vector;
  // LightDataVector good_observations;
  for (DataPairMap::iterator tr_it = data_pair_map.begin(); tr_it != data_pair_map.end(); tr_it++) {
    // Verify if the tracker is in the body frame
    if (body_transforms.find(tr_it->first) == body_transforms.end()) {
      ROS_INFO_STREAM("Tracker " << tr_it->first << " not in body frame " << calibration_body << " - IGNORING");
      continue;
    }
    // Setting all the necessary variables for a single tracker
    Extrinsics extrinsics;
    extrinsics.size = ViveUtils::ConvertExtrinsics(calibration.trackers[tr_it->first], extrinsics.positions);
    LightData observations;
    for (SweepVec::iterator sw_it = tr_it->second.first.begin();
      sw_it != tr_it->second.first.end(); sw_it++) {
      size_t counter = 0;
      observations[sw_it->lighthouse].axis[sw_it->axis].lights = sw_it->lights;
      if (observations[sw_it->lighthouse].axis[HORIZONTAL].lights.size() > 2
        && observations[sw_it->lighthouse].axis[VERTICAL].lights.size() > 2) {
        counter++;
        SolvedPose solvedpose;

        for (size_t i = 0; i < 6; i++) solvedpose.transform[i] = calibrate::start_pose[i];
        std::mutex solvemutex;
        std::string auxstring;
        Lighthouse lh_extrinsics;
        if (calibration.lighthouses.find(sw_it->lighthouse) != calibration.lighthouses.end()) {
          lh_extrinsics = calibration.lighthouses[sw_it->lighthouse];
        }
        if (ComputeTransform(observations[sw_it->lighthouse],
          &solvedpose,
          &auxstring,
          &extrinsics,
          &solvemutex,
          &lh_extrinsics)) {
          Eigen::Vector3d tmV = Eigen::Vector3d(solvedpose.transform[3],
            solvedpose.transform[4],
            solvedpose.transform[5]);

          PoseVM tmp_pose;
          tmp_pose.second =
            Eigen::AngleAxisd(tmV.norm(), tmV/tmV.norm()).toRotationMatrix().transpose();
          tmV = Eigen::Vector3d(solvedpose.transform[0], solvedpose.transform[1], solvedpose.transform[2]);
          tmp_pose.first = - tmp_pose.second * tmV;
          // std::cout << "P: " << tmp_pose.first << std::endl;
          // Saving all poses in a vector
          poses_vector[tr_it->first][sw_it->lighthouse].push_back(tmp_pose);
          // std::cout << "R: " << tmp_pose.second << std::endl;
          // Saving all the good observation
        }
      }
    }
    // Averaging the poses
    PoseVV averaged_pose(Eigen::Vector3d::Zero(), Eigen::Vector4d::Zero());
    PoseVV master_pose;
    for (PosesMap::iterator lh_it = poses_vector[tr_it->first].begin();
      lh_it != poses_vector[tr_it->first].end(); lh_it++) {
      for (Poses::iterator po_it = lh_it->second.begin();
        po_it != lh_it->second.end(); po_it++) {
        // Conversion of quaternions
        Eigen::Quaterniond tmp_Q = Eigen::Quaterniond(po_it->second);
        Eigen::Vector4d tmp_V4 = Eigen::Vector4d(tmp_Q.w(),
          tmp_Q.x(),
          tmp_Q.y(),
          tmp_Q.z());
        if (po_it == lh_it->second.begin()) {
          master_pose.first = po_it->first;
          master_pose.second = tmp_V4;
          averaged_pose = master_pose;
        } else {
          averaged_pose.first += po_it->first;
          if ((tmp_V4.transpose() * master_pose.second)(0) < 0)
            tmp_V4 = -tmp_V4;
          averaged_pose.second += tmp_V4;
        }
      }
      averaged_pose.first = averaged_pose.first / static_cast<double>(lh_it->second.size());
      averaged_pose.second = averaged_pose.second / static_cast<double>(lh_it->second.size());
      averaged_pose.second.normalize();
      (*poses)[lh_it->first][tr_it->first].first = averaged_pose.first;
      (*poses)[lh_it->first][tr_it->first].second =
        Eigen::Quaterniond(averaged_pose.second(0),
          averaged_pose.second(1),
          averaged_pose.second(2),
          averaged_pose.second(3)).toRotationMatrix();
      std::cout << lh_it->first << std::endl;
      // std::cout << "avg P: " << (*poses)[lh_it->first][tr_it->first].first << std::endl;
      // std::cout << "avg R: " << (*poses)[lh_it->first][tr_it->first].second << std::endl;
      lh_it->second.clear();
    }
    // std::cout <<  << std::endl;
  }
  // exit(0);
  return true;
}

// Converts the poses of the lighthouses to the world frame
bool GetLhTransformInW(PoseLighthouses * world_lighthouses,
  PoseTrackers body_transforms,
  PoseMap poses ) {
  if (poses.size() == 0) {
    ROS_FATAL("No trackers avaliable for calibration");
    return false;
  }
  for (PoseMap::iterator lh_it = poses.begin(); lh_it != poses.end(); lh_it++) {
    // Initialization for averaging
    PoseVQ lTb;
    lTb.first = Eigen::Vector3d::Zero();
    lTb.second = Eigen::Quaterniond(0, 0, 0, 0);
    Eigen::Quaterniond auxQ = Eigen::Quaterniond(0, 0, 0, 0);
    Eigen::Quaterniond master = Eigen::Quaterniond(0, 0, 0, 0);
    double body_trackers = 0;
    for (PoseTrackers::iterator tr_it = lh_it->second.begin(); tr_it != lh_it->second.end(); tr_it++) {
      // Prevent use of non-body trackers
      if (body_transforms.find(tr_it->first) == body_transforms.end()) {
        continue;
      }
      // Sum all poses
      lTb.first += body_transforms[tr_it->first].second * poses[lh_it->first][tr_it->first].first
        + body_transforms[tr_it->first].first;
      // Sum all quaternions and converting to master direction
      auxQ = Eigen::Quaterniond(body_transforms[tr_it->first].second *
        poses[lh_it->first][tr_it->first].second);
      if ( lTb.second.w() == 0
        && lTb.second.x() == 0
        && lTb.second.y() == 0
        && lTb.second.z() == 0 ) {
        master = auxQ;
      }
      if ( master.w() * auxQ.w()
        + master.x() * auxQ.x()
        + master.y() * auxQ.y()
        + master.z() * auxQ.z() >= 0 ) {
        lTb.second.x() += auxQ.x();
        lTb.second.y() += auxQ.y();
        lTb.second.z() += auxQ.z();
        lTb.second.w() += auxQ.w();
      } else {
        lTb.second.x() -= auxQ.x();
        lTb.second.y() -= auxQ.y();
        lTb.second.z() -= auxQ.z();
        lTb.second.w() -= auxQ.w();
      }
      body_trackers++;
    }
    if (body_trackers == 0) {
      continue;
    }
    // Saving the averaged poses
    (*world_lighthouses)[lh_it->first].first = lTb.first / body_trackers;
    lTb.second.normalize();
    (*world_lighthouses)[lh_it->first].second = lTb.second.toRotationMatrix();
    std::cout << lh_it->first <<std::endl;
    // std::cout << "P = " << (*world_lighthouses)[lh_it->first].first << std::endl;
    // std::cout << "R = " << (*world_lighthouses)[lh_it->first].second << std::endl;
  }
  return true;
}

// Optimizes the solution
bool BundleObservations(PoseLighthouses * world_lighthouses,
  PoseTrackers body_transforms,
  DataPairMap data_pair_map,
  Calibration calibration) {
  // if (data_pair_map.size() > 9 * (*world_lighthouses).size()) {
  std::map<std::string, double[5]> lh_horizontal_extrinsics;
  std::map<std::string, double[5]> lh_vertical_extrinsics;
  std::vector<ceres::ResidualBlockId> h_residual_block_ids;
  std::vector<ceres::ResidualBlockId> v_residual_block_ids;

  if (true) {
    ceres::Problem problem;
    std::map<std::string, double[6]> bundle_lighthouses_world;
    std::map<std::string, Extrinsics> extrinsics;
    for (DataPairMap::iterator tr_it = data_pair_map.begin(); tr_it != data_pair_map.end(); tr_it++) {
      // Check if sweep is of tracker in body
      if (body_transforms.find(tr_it->first) == body_transforms.end())
        continue;
      extrinsics[tr_it->first].size = ViveUtils::ConvertExtrinsics(
        calibration.trackers[tr_it->first],
        extrinsics[tr_it->first].positions);
      for (SweepVec::iterator sw_it = tr_it->second.first.begin();
        sw_it != tr_it->second.first.end(); sw_it++) {
        LightVec light_vec;
        light_vec = sw_it->lights;
        if (light_vec.size() == 0) {
          continue;
        }

        bool correction = false;
        // double lh_horizontal_extrinsics[5] = {0,0,0,0,0};
        // double lh_vertical_extrinsics[5] = {0,0,0,0,0};
        // auto lh_it = calibration.lighthouses.begin();
        if (calibration.lighthouses.find(sw_it->lighthouse) != calibration.lighthouses.end()
          && CORRECTION) {
          // std::cout << "correctionON" << std::endl;
          correction = true;
        } else {
          // std::cout << "correctionOFF" << std::endl;
        }
        // correction = false;

        // World in lighthouse frame
        PoseVM lighthouse_world;
        lighthouse_world.second = (*world_lighthouses)[sw_it->lighthouse].second.transpose();
        lighthouse_world.first = -lighthouse_world.second * (*world_lighthouses)[sw_it->lighthouse].first;
        Eigen::AngleAxisd tmp_AA =
          Eigen::AngleAxisd(lighthouse_world.second);
        bundle_lighthouses_world[sw_it->lighthouse][0] = lighthouse_world.first(0);
        bundle_lighthouses_world[sw_it->lighthouse][1] = lighthouse_world.first(1);
        bundle_lighthouses_world[sw_it->lighthouse][2] = lighthouse_world.first(2);
        bundle_lighthouses_world[sw_it->lighthouse][3] = tmp_AA.axis()(0)*tmp_AA.angle();
        bundle_lighthouses_world[sw_it->lighthouse][4] = tmp_AA.axis()(1)*tmp_AA.angle();
        bundle_lighthouses_world[sw_it->lighthouse][5] = tmp_AA.axis()(2)*tmp_AA.angle();
        // std::cout << sw_it->lighthouse << " - "
        //   << bundle_lighthouses_world[sw_it->lighthouse][0] << ", "
        //   << bundle_lighthouses_world[sw_it->lighthouse][1] << ", "
        //   << bundle_lighthouses_world[sw_it->lighthouse][2] << ", "
        //   << bundle_lighthouses_world[sw_it->lighthouse][3] << ", "
        //   << bundle_lighthouses_world[sw_it->lighthouse][4] << ", "
        //   << bundle_lighthouses_world[sw_it->lighthouse][5] << std::endl;
        if (sw_it->axis == HORIZONTAL) {
          ceres::DynamicAutoDiffCostFunction<CalibHorizontalAngle, 4> * horizontal_cost =
            new ceres::DynamicAutoDiffCostFunction<CalibHorizontalAngle, 4>
            (new CalibHorizontalAngle(light_vec, body_transforms[tr_it->first], correction));
          horizontal_cost->AddParameterBlock(6);
          horizontal_cost->AddParameterBlock(3 * extrinsics[tr_it->first].size);
          horizontal_cost->AddParameterBlock(1);
          horizontal_cost->AddParameterBlock(5);
          horizontal_cost->SetNumResiduals(light_vec.size());


          if (correction) {
            lh_horizontal_extrinsics[sw_it->lighthouse][0] =
              calibration.lighthouses[sw_it->lighthouse].horizontal_motor.phase;
            lh_horizontal_extrinsics[sw_it->lighthouse][1] =
              calibration.lighthouses[sw_it->lighthouse].horizontal_motor.tilt;
            lh_horizontal_extrinsics[sw_it->lighthouse][2] =
              calibration.lighthouses[sw_it->lighthouse].horizontal_motor.gib_phase;
            lh_horizontal_extrinsics[sw_it->lighthouse][3] =
              calibration.lighthouses[sw_it->lighthouse].horizontal_motor.gib_magnitude;
            lh_horizontal_extrinsics[sw_it->lighthouse][4] =
              calibration.lighthouses[sw_it->lighthouse].horizontal_motor.curve;
          } else {
            lh_horizontal_extrinsics[sw_it->lighthouse][0] = 0.0;
            lh_horizontal_extrinsics[sw_it->lighthouse][1] = 0.0;
            lh_horizontal_extrinsics[sw_it->lighthouse][2] = 0.0;
            lh_horizontal_extrinsics[sw_it->lighthouse][3] = 0.0;
            lh_horizontal_extrinsics[sw_it->lighthouse][4] = 0.0;
          }
          // std::cout << "HExt Out "
          //   << lh_horizontal_extrinsics[sw_it->lighthouse][0] << ", "
          //   << lh_horizontal_extrinsics[sw_it->lighthouse][1] << ", "
          //   << lh_horizontal_extrinsics[sw_it->lighthouse][2] << ", "
          //   << lh_horizontal_extrinsics[sw_it->lighthouse][3] << ", "
          //   << lh_horizontal_extrinsics[sw_it->lighthouse][4] << std::endl;
          ceres::ResidualBlockId hrb = problem.AddResidualBlock(horizontal_cost,
            NULL,
            bundle_lighthouses_world[sw_it->lighthouse],
            extrinsics[tr_it->first].positions,
            &(extrinsics[tr_it->first].radius),
            lh_horizontal_extrinsics[sw_it->lighthouse]);
          h_residual_block_ids.push_back(hrb);

          // problem.SetParameterBlockConstant(lh_horizontal_extrinsics[sw_it->lighthouse]);

        } else if (sw_it->axis == VERTICAL) {
          ceres::DynamicAutoDiffCostFunction<CalibVerticalAngle, 4> * vertical_cost =
            new ceres::DynamicAutoDiffCostFunction<CalibVerticalAngle, 4>
            (new CalibVerticalAngle(light_vec, body_transforms[tr_it->first], correction));
          vertical_cost->AddParameterBlock(6);
          vertical_cost->AddParameterBlock(3 * extrinsics[tr_it->first].size);
          vertical_cost->AddParameterBlock(1);
          vertical_cost->AddParameterBlock(5);
          vertical_cost->SetNumResiduals(light_vec.size());

          if (correction) {
            lh_vertical_extrinsics[sw_it->lighthouse][0] =
              calibration.lighthouses[sw_it->lighthouse].vertical_motor.phase;
            lh_vertical_extrinsics[sw_it->lighthouse][1] =
              calibration.lighthouses[sw_it->lighthouse].vertical_motor.tilt;
            lh_vertical_extrinsics[sw_it->lighthouse][2] =
              calibration.lighthouses[sw_it->lighthouse].vertical_motor.gib_phase;
            lh_vertical_extrinsics[sw_it->lighthouse][3] =
              calibration.lighthouses[sw_it->lighthouse].vertical_motor.gib_magnitude;
            lh_vertical_extrinsics[sw_it->lighthouse][4] =
              calibration.lighthouses[sw_it->lighthouse].vertical_motor.curve;
          } else {
            lh_vertical_extrinsics[sw_it->lighthouse][0] = 0.0;
            lh_vertical_extrinsics[sw_it->lighthouse][1] = 0.0;
            lh_vertical_extrinsics[sw_it->lighthouse][2] = 0.0;
            lh_vertical_extrinsics[sw_it->lighthouse][3] = 0.0;
            lh_vertical_extrinsics[sw_it->lighthouse][4] = 0.0;
          }
          // std::cout << "VExt Out "
          //   << lh_vertical_extrinsics[sw_it->lighthouse][0] << ", "
          //   << lh_vertical_extrinsics[sw_it->lighthouse][1] << ", "
          //   << lh_vertical_extrinsics[sw_it->lighthouse][2] << ", "
          //   << lh_vertical_extrinsics[sw_it->lighthouse][3] << ", "
          //   << lh_vertical_extrinsics[sw_it->lighthouse][4] << std::endl;

          ceres::ResidualBlockId vrb = problem.AddResidualBlock(vertical_cost,
            NULL,
            bundle_lighthouses_world[sw_it->lighthouse],
            extrinsics[tr_it->first].positions,
            &(extrinsics[tr_it->first].radius),
            lh_vertical_extrinsics[sw_it->lighthouse]);
          v_residual_block_ids.push_back(vrb);

          // problem.SetParameterBlockConstant(lh_vertical_extrinsics[sw_it->lighthouse]);
        }
      }
      problem.SetParameterBlockConstant(extrinsics[tr_it->first].positions);
      problem.SetParameterBlockConstant(&(extrinsics[tr_it->first].radius));
    }
    for (auto lH_it = lh_horizontal_extrinsics.begin();
      lH_it != lh_horizontal_extrinsics.end(); lH_it++) {
      problem.SetParameterBlockConstant(lH_it->second);
    }
    for (auto lV_it = lh_vertical_extrinsics.begin();
      lV_it != lh_vertical_extrinsics.end(); lV_it++) {
      problem.SetParameterBlockConstant(lV_it->second);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = true;
    // options.minimizer_type = ceres::LINE_SEARCH;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // std::cout << "HERE5" << std::endl;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    // std::cout << "HERE6" << std::endl;

    for (auto lh_it = bundle_lighthouses_world.begin();
      lh_it != bundle_lighthouses_world.end(); lh_it++) {
      // Eigen::Vector3d wPl(lh_it->second[0], lh_it->second[1], lh_it->second[2]);
      // Eigen::Vector3d wAAl(lh_it->second[3], lh_it->second[4], lh_it->second[5]);
      // Eigen::Matrix3d wRl = Eigen::AngleAxisd(wAAl.norm(), wAAl / wAAl.norm()).toRotationMatrix();

      // Eigen::AngleAxisd lAAw(wRl.transpose());
      // Eigen::Vector3d lPw = -wRl.transpose() * wPl;

      // std::cout << lh_it->first << " - NEW POSE: "
      //   << lPw(0) << ", "
      //   << lPw(1) << ", "
      //   << lPw(2) << ", "
      //   << lAAw.axis()(0) * lAAw.angle() << ", "
      //   << lAAw.axis()(1) * lAAw.angle() << ", "
      //   << lAAw.axis()(2) * lAAw.angle() << std::endl;

      std::cout << lh_it->first << " - POSE: "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
    }

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

    for (auto lh_it = bundle_lighthouses_world.begin();
      lh_it != bundle_lighthouses_world.end(); lh_it++) {
        Eigen::Vector3d tmp_V = Eigen::Vector3d(lh_it->second[3], lh_it->second[4], lh_it->second[5]);
        (*world_lighthouses)[lh_it->first].second =
          Eigen::AngleAxisd(tmp_V.norm(), tmp_V/tmp_V.norm()).toRotationMatrix().transpose();
        (*world_lighthouses)[lh_it->first].first =
          - (*world_lighthouses)[lh_it->first].second *
          Eigen::Vector3d(lh_it->second[0], lh_it->second[1], lh_it->second[2]);
    }
  }
  return true;
}

bool GetLhTransformInVive(Calibration * calibration,
  PoseLighthouses world_lighthouses) {
  PoseVM vive_pose;               // pose of the vive frame in the world frame
  PoseVQ vive_lighthouse;         // pose of the tracker in the vive frame
  Eigen::Quaterniond vive_quaternion;
  if (world_lighthouses.size() == 0) {
    ROS_FATAL("No lighthouses in sight");
    return false;
  }
  // Choosing the master lighthouse
  vive_pose.first = world_lighthouses.begin()->second.first;
  vive_pose.second = world_lighthouses.begin()->second.second;
  vive_quaternion = Eigen::Quaterniond(vive_pose.second);
  // Saving the world-vive transform in the calibration file
  (*calibration).environment.vive.translation.x = vive_pose.first(0);
  (*calibration).environment.vive.translation.y = vive_pose.first(1);
  (*calibration).environment.vive.translation.z = vive_pose.first(2);
  (*calibration).environment.vive.rotation.x = vive_quaternion.x();
  (*calibration).environment.vive.rotation.y = vive_quaternion.y();
  (*calibration).environment.vive.rotation.z = vive_quaternion.z();
  (*calibration).environment.vive.rotation.w = vive_quaternion.w();

  // std::cout << "VIVE " << vive_pose.first(0) << ", "
  //   << vive_pose.first(1) << ", "
  //   << vive_pose.first(2) << ", "
  //   << vive_quaternion.w() << ", "
  //   << vive_quaternion.x() << ", "
  //   << vive_quaternion.y() << ", "
  //   << vive_quaternion.z() << std::endl;

  // Clear past lighthouse data
  (*calibration).environment.lighthouses.clear();
  // Compute all lighthouse vive poses and saving them
  for (PoseLighthouses::iterator lh_it = world_lighthouses.begin();
    lh_it != world_lighthouses.end(); lh_it++) {
    vive_lighthouse.first = vive_pose.second.transpose() * lh_it->second.first
      - vive_pose.second.transpose() * vive_pose.first;
    vive_lighthouse.second = Eigen::Quaterniond(vive_pose.second.transpose() * lh_it->second.second);

    Transform vive_tf_lh;
    vive_tf_lh.parent_frame = "vive";
    vive_tf_lh.child_frame = lh_it->first;
    vive_tf_lh.translation.x = vive_lighthouse.first(0);
    vive_tf_lh.translation.y = vive_lighthouse.first(1);
    vive_tf_lh.translation.z = vive_lighthouse.first(2);
    vive_tf_lh.rotation.w = vive_lighthouse.second.w();
    vive_tf_lh.rotation.x = vive_lighthouse.second.x();
    vive_tf_lh.rotation.y = vive_lighthouse.second.y();
    vive_tf_lh.rotation.z = vive_lighthouse.second.z();

  // std::cout << "LH " << vive_lighthouse.first(0) << ", "
  //   << vive_lighthouse.first(1) << ", "
  //   << vive_lighthouse.first(2) << ", "
  //   << vive_lighthouse.second.w() << ", "
  //   << vive_lighthouse.second.x() << ", "
  //   << vive_lighthouse.second.y() << ", "
  //   << vive_lighthouse.second.z() << std::endl;

    (*calibration).environment.lighthouses[lh_it->first] = vive_tf_lh;
  }
  return true;
}

bool GetGravity(Calibration * cal,
  DataPairMap data) {

  double g_count = 0;
  Eigen::Vector3d vG(0.0, 0.0, 0.0);
  // Iterate trackers
  for (auto tracker_data : data) {
    // Iterate inertial messages
    double t_count = 0;
    Eigen::Vector3d tracker_iG(0.0, 0.0, 0.0);
    for (auto msg : tracker_data.second.second) {
      Eigen::Vector3d sample_iG(msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z);
      tracker_iG += sample_iG;
      t_count++;
    }
    // Averaging
    tracker_iG = tracker_iG / t_count;
    // Convert to vive frame
    // This is the same as tQv
    Eigen::Quaterniond wQv(cal->environment.vive.rotation.w,
      cal->environment.vive.rotation.x,
      cal->environment.vive.rotation.y,
      cal->environment.vive.rotation.z);
    Eigen::Quaterniond lQi(cal->trackers[tracker_data.first].imu_transform.rotation.w,
      cal->trackers[tracker_data.first].imu_transform.rotation.x,
      cal->trackers[tracker_data.first].imu_transform.rotation.y,
      cal->trackers[tracker_data.first].imu_transform.rotation.z);
    Eigen::Vector3d tracker_vG = wQv.toRotationMatrix().transpose() *
      lQi.toRotationMatrix() * tracker_iG;
    vG += tracker_vG;
    g_count++;
  }
  vG = vG / g_count;
  cal->environment.gravity.x = vG(0);
  cal->environment.gravity.y = vG(1);
  cal->environment.gravity.z = vG(2);
  return true;
}



// Worker thread
void ViveCalibrate::WorkerThread(CallbackFn cb,
  std::mutex * calibration_mutex,
  DataPairMap data_pair_map,
  Calibration calibration) {
  PoseLighthouses world_lighthouses;
  PoseTrackers body_transforms;
  std::string calibration_body;
  PoseMap poses;

  // Prevent data from being modified outside this thread
  calibration_mutex->lock();

  // Get the body frames
  std::cout << "GetBodyTransformsInW" << std::endl;
  if (!GetBodyTransformsInW(&body_transforms,
    &calibration_body,
    calibration)) {
    calibration_mutex->unlock();
    return;
  }
  // Now we have wRt and wPt

  // Organize the data of each tracker in groups of lighthouses and axis and solve
  std::cout << "GetLhTransformsInTr" << std::endl;
  if (!GetLhTransformsInTr(&poses,
    data_pair_map,
    body_transforms,
    calibration,
    calibration_body)) {
    calibration_mutex->unlock();
    return;
  }
  // Now we have tRl and tPl

  // Convert from the pose of the lighthouse in the tracker frame to world frame
  std::cout << "GetLhTransformInW" << std::endl;
  if (!GetLhTransformInW(&world_lighthouses,
    body_transforms,
    poses)) {
    calibration_mutex->unlock();
    return;
  }
  // Now we have wRl and wPl

  // Optimizing the solution
  std::cout << "BundleObservations" << std::endl;
  if (!BundleObservations(&world_lighthouses,
    body_transforms,
    data_pair_map,
    calibration)) {
    calibration_mutex->unlock();
    return;
  }

  // Choose the vive frame and convert everything to this frame //
  std::cout << "GetLhTransformInVive" << std::endl;
  if (!GetLhTransformInVive(&calibration,
    world_lighthouses)) {
    calibration_mutex->unlock();
    return;
  }

  // // Get the gravity vector
  // calibration_mutex->unlock();
  // std::cout << "GetGravity" << std::endl;
  // if (!GetGravity(&calibration,
  //   data_pair_map)) {
  //   calibration_mutex->unlock();
  //   return;
  // }

  // Callback
  std::cout << "cb" << std::endl;
  cb(calibration);

  std::cout << "return" << std::endl;
  return;
}