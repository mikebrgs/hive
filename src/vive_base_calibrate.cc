#include <hive/vive_base_calibrate.h>

// Constructor just sets the callback function
BaseCalibrate::BaseCalibrate(Calibration & calibration) {
  calibration_ = calibration;
}

// Reset
bool BaseCalibrate::Reset() {
  data_pair_map_.clear();
  return true;
}

// Add an IMU measurement
bool BaseCalibrate::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // do nothing
  return true;
}

// Add a light measurement
bool BaseCalibrate::AddLight(const hive::ViveLight::ConstPtr& msg) {
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

  // Store the data
  if (msg->samples.size() > 0) {
    data_pair_map_[msg->header.frame_id].first.push_back(sweep);
  }

  return true;
}


// Start solving in a parallel thread
// bool BaseCalibrate::Solve() {
  // Wait for thread to join, in case of old solution
  // std::thread thread = std::thread(BaseCalibrate::WorkerThread,
  //   data_pair_map_,
  //   calibration_);
  // thread.join();
//   WorkerThread(data_pair_map_,
//     calibration_);
//   return true;
// }

Calibration BaseCalibrate::GetCalibration() {
  return calibration_;
}


struct RayHorizontalAngle{
  explicit RayHorizontalAngle(LightVec horizontal_observations) :
  horizontal_observations_(horizontal_observations) {}

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
      ang = atan(x);
      residual[i] = T(horizontal_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec horizontal_observations_;
};

struct RayVerticalAngle{
  explicit RayVerticalAngle(LightVec vertical_observations) :
  vertical_observations_(vertical_observations){}

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
      T y = (lPs(1)/lPs(2)); // Vertical angle
      ang = atan(y);
      residual[i] = T(vertical_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec vertical_observations_;
};

bool ComputeTransform(AxisLightVec observations,
  SolvedPose * pose_tracker,
  std::string * last_lh_pose,
  Extrinsics * extrinsics) {
  ceres::Problem problem;
  double pose[6], lighthouseZeros[6];
  for (int i = 0; i < 6; i++) {
    pose[i] = pose_tracker->transform[i];
    lighthouseZeros[i] = 0.0;
  }


  ceres::DynamicAutoDiffCostFunction<RayVerticalAngle, 4> * vertical_cost =
    new ceres::DynamicAutoDiffCostFunction<RayVerticalAngle, 4>
    (new RayVerticalAngle(observations.axis[VERTICAL].lights));
  vertical_cost->AddParameterBlock(6);
  vertical_cost->AddParameterBlock(3 * extrinsics->size);
  vertical_cost->AddParameterBlock(6);
  vertical_cost->AddParameterBlock(1);
  vertical_cost->SetNumResiduals(observations.axis[VERTICAL].lights.size());

  ceres::ResidualBlockId vrb = problem.AddResidualBlock(vertical_cost,
    NULL,
    pose,
    extrinsics->positions,
    lighthouseZeros,
    &(extrinsics->radius));
  std::vector<ceres::ResidualBlockId> v_residual_block_ids;
  v_residual_block_ids.push_back(vrb);

  ceres::DynamicAutoDiffCostFunction<RayHorizontalAngle, 4> * horizontal_cost =
    new ceres::DynamicAutoDiffCostFunction<RayHorizontalAngle, 4>
    (new RayHorizontalAngle(observations.axis[HORIZONTAL].lights));
  horizontal_cost->AddParameterBlock(6);
  horizontal_cost->AddParameterBlock(3 * extrinsics->size);
  horizontal_cost->AddParameterBlock(6);
  horizontal_cost->AddParameterBlock(1);
  horizontal_cost->SetNumResiduals(observations.axis[HORIZONTAL].lights.size());

  ceres::ResidualBlockId hrb = problem.AddResidualBlock(horizontal_cost,
    NULL,
    pose,
    extrinsics->positions,
    lighthouseZeros,
    &(extrinsics->radius));
  std::vector<ceres::ResidualBlockId> h_residual_block_ids;
  h_residual_block_ids.push_back(hrb);

  problem.SetParameterBlockConstant(extrinsics->positions);
  problem.SetParameterBlockConstant(lighthouseZeros);
  problem.SetParameterBlockConstant(&(extrinsics->radius));

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 1.0;

  ceres::Solve(options, &problem, &summary);
  {
    std::cout << "CT: " << summary.final_cost << ", " << summary.num_residual_blocks << ", " << summary.num_residuals << " - "
      << pose[0] << ", "
      << pose[1] << ", "
      << pose[2] << ", "
      << pose[3] << ", "
      << pose[4] << ", "
      << pose[5] << std::endl;
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
  for (int i = 0; i < 6; i++) {
    pose_tracker->transform[i] = pose[i];
  }
  pose_tracker->lighthouse = observations.lighthouse;
  pose_tracker->valid = true;
  // pose_tracker->stamp = ros::Time::now();
  *last_lh_pose = observations.lighthouse;

  return true;
}


struct CalibHorizontalAngle{
  explicit CalibHorizontalAngle(LightVec horizontal_observations, PoseVM world_tracker_transforms) :
  horizontal_observations_(horizontal_observations),
  world_tracker_transforms_(world_tracker_transforms) {}

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

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      ang = atan(x);

      residual[i] = T(horizontal_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec horizontal_observations_;
  PoseVM world_tracker_transforms_;
};

struct CalibVerticalAngle{
  explicit CalibVerticalAngle(LightVec vertical_observations, PoseVM world_tracker_transforms) :
  vertical_observations_(vertical_observations),
  world_tracker_transforms_(world_tracker_transforms) {}

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

      T ang; // The final angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      ang = atan(y);

      residual[i] = T(vertical_observations_[i].angle) - ang;
    }

    return true;
  }

 private:
  LightVec vertical_observations_;
  PoseVM world_tracker_transforms_;
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
  Quat.normalize();
  body_offset.second = Quat.toRotationMatrix();

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


    // Saving the transforms of the tracker frame in the body frame
    Eigen::Vector3d tmP;
    Eigen::Matrix3d tmR;
    tmR = (*body_transforms)[bd_it->second.parent_frame].second.transpose();
    (*body_transforms)[bd_it->second.parent_frame].second = body_offset.second * tmR;
    tmP = - tmR * (*body_transforms)[bd_it->second.parent_frame].first;
    (*body_transforms)[bd_it->second.parent_frame].first = body_offset.second * tmP + body_offset.first;
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
        std::string auxstring;
        Lighthouse lh_extrinsics;
        if (calibration.lighthouses.find(sw_it->lighthouse) != calibration.lighthouses.end()) {
          lh_extrinsics = calibration.lighthouses[sw_it->lighthouse];
        }
        if (ComputeTransform(observations[sw_it->lighthouse],
          &solvedpose,
          &auxstring,
          &extrinsics)) {
          Eigen::Vector3d tmV = Eigen::Vector3d(solvedpose.transform[3],
            solvedpose.transform[4],
            solvedpose.transform[5]);

          PoseVM tmp_pose;
          tmp_pose.second =
            Eigen::AngleAxisd(tmV.norm(), tmV/tmV.norm()).toRotationMatrix().transpose();
          tmV = Eigen::Vector3d(solvedpose.transform[0], solvedpose.transform[1], solvedpose.transform[2]);
          tmp_pose.first = - tmp_pose.second * tmV;
          // Saving all poses in a vector
          poses_vector[tr_it->first][sw_it->lighthouse].push_back(tmp_pose);
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
  }
  return true;
}

// Optimizes the solution
bool BundleObservations(PoseLighthouses * world_lighthouses,
  PoseTrackers body_transforms,
  DataPairMap data_pair_map,
  Calibration calibration) {
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
        if (sw_it->axis == HORIZONTAL) {
          ceres::DynamicAutoDiffCostFunction<CalibHorizontalAngle, 4> * horizontal_cost =
            new ceres::DynamicAutoDiffCostFunction<CalibHorizontalAngle, 4>
            (new CalibHorizontalAngle(light_vec, body_transforms[tr_it->first]));
          horizontal_cost->AddParameterBlock(6);
          horizontal_cost->AddParameterBlock(3 * extrinsics[tr_it->first].size);
          horizontal_cost->AddParameterBlock(1);
          horizontal_cost->SetNumResiduals(light_vec.size());


          ceres::ResidualBlockId hrb = problem.AddResidualBlock(horizontal_cost,
            NULL,
            bundle_lighthouses_world[sw_it->lighthouse],
            extrinsics[tr_it->first].positions,
            &(extrinsics[tr_it->first].radius));
          h_residual_block_ids.push_back(hrb);


        } else if (sw_it->axis == VERTICAL) {
          ceres::DynamicAutoDiffCostFunction<CalibVerticalAngle, 4> * vertical_cost =
            new ceres::DynamicAutoDiffCostFunction<CalibVerticalAngle, 4>
            (new CalibVerticalAngle(light_vec, body_transforms[tr_it->first]));
          vertical_cost->AddParameterBlock(6);
          vertical_cost->AddParameterBlock(3 * extrinsics[tr_it->first].size);
          vertical_cost->AddParameterBlock(1);
          vertical_cost->SetNumResiduals(light_vec.size());

          ceres::ResidualBlockId vrb = problem.AddResidualBlock(vertical_cost,
            NULL,
            bundle_lighthouses_world[sw_it->lighthouse],
            extrinsics[tr_it->first].positions,
            &(extrinsics[tr_it->first].radius));
          v_residual_block_ids.push_back(vrb);

        }
      }
      problem.SetParameterBlockConstant(extrinsics[tr_it->first].positions);
      problem.SetParameterBlockConstant(&(extrinsics[tr_it->first].radius));
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

      std::cout << lh_it->first << " - POSE: "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
    }

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

    (*calibration).environment.lighthouses[lh_it->first] = vive_tf_lh;
  }
  return true;
}



// Worker thread
bool BaseCalibrate::Solve() {
  PoseLighthouses world_lighthouses;
  PoseTrackers body_transforms;
  std::string calibration_body;
  PoseMap poses;

  // Prevent data from being modified outside this thread

  // Get the body frames
  std::cout << "GetBodyTransformsInW" << std::endl;
  if (!GetBodyTransformsInW(&body_transforms,
    &calibration_body,
    calibration_)) {
    return false;
}
  // Now we have wRt and wPt

  // Organize the data of each tracker in groups of lighthouses and axis and solve
  std::cout << "GetLhTransformsInTr" << std::endl;
  if (!GetLhTransformsInTr(&poses,
    data_pair_map_,
    body_transforms,
    calibration_,
    calibration_body)) {
    return false;
}
  // Now we have tRl and tPl

  // Convert from the pose of the lighthouse in the tracker frame to world frame
  std::cout << "GetLhTransformInW" << std::endl;
  if (!GetLhTransformInW(&world_lighthouses,
    body_transforms,
    poses)) {
    return false;
}
  // Now we have wRl and wPl

  // Optimizing the solution
  std::cout << "BundleObservations" << std::endl;
  if (!BundleObservations(&world_lighthouses,
    body_transforms,
    data_pair_map_,
    calibration_)) {
    return false;
}

  // Choose the vive frame and convert everything to this frame //
  std::cout << "GetLhTransformInVive" << std::endl;
  if (!GetLhTransformInVive(&calibration_,
    world_lighthouses)) {
    return false;
}

  return true;
}

int main(int argc, char ** argv)
{
  // Data
  Calibration calibration;


  ros::init(argc, argv, "hive_baseline_calibrator");
  ros::NodeHandle nh;

  // Read bag with data
  if (argc < 2) {
    std::cout << "Usage: ... hive_calibrator name_of_read_bag.bag" << std::endl;
    return -1;
  }
  rosbag::Bag rbag;
  rosbag::View view;
  std::string read_bag(argv[1]);
  rbag.open(read_bag, rosbag::bagmode::Read);

  // Start JSON parser
  JsonParser jp = JsonParser(HIVE_CONFIG_FILE);
  // Get current calibration
  if (!ViveUtils::ReadConfig(HIVE_BASE_CALIBRATION_FILE, &calibration)) {
    jp.GetCalibration(&calibration);
    ROS_WARN("Reading JSON file.");
  } else {
    jp.GetBody(&calibration);
    ROS_INFO("Read calibration file.");
  }
  std::cout << "Bodies: " <<
    calibration.environment.bodies.size() << std::endl;

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    calibration.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    calibration.SetTrackers(*vt);
    // TODO set up solver.
  }
  ROS_INFO("Trackers' setup complete.");

  BaseCalibrate calibrator(calibration);

  // Light data
  size_t counter = 0;
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    calibrator.AddLight(vl);
    counter++;
    if (counter == 10)
      break;
  }
  rbag.close();

  calibrator.Solve();
  ViveUtils::WriteConfig(HIVE_BASE_CALIBRATION_FILE,
    calibrator.GetCalibration());

  ros::shutdown();
  return 0;
}