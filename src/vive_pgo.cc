#include <hive/vive_pgo.h>

#define TRUST 0.4
// #define TRUST 1.6

enum DataType {imu, light};

PoseGraph::PoseGraph(Environment environment,
  Tracker tracker,
  std::map<std::string, Lighthouse> lighthouses,
  size_t window,
  double trust,
  bool correction) {
  if (window < 2) {
    std::cout << "Bad window size. Using 2." << std::endl;
    window_ = 2;
  } else {
    window_ = window;
  }
  valid_ = false;
  lastposewasimu_ = false;
  correction_ = correction;
  trust_ = trust;
  tracker_ = tracker;
  environment_ = environment;
  lighthouses_ = lighthouses;
  return;
}

PoseGraph::PoseGraph() {
  valid_ = true;
  return;
}

PoseGraph::~PoseGraph() {
  // pass
  return;
}


bool PoseGraph::Valid() {
  // Parameters to validate pose
  double cost = 0;
  double sample_counter = 0;
  // Pose of the tracker in the vive frame
  Eigen::Vector3d vPt(pose_.transform.translation.x,
    pose_.transform.translation.y,
    pose_.transform.translation.z);
  Eigen::Quaterniond vQt(pose_.transform.rotation.w,
    pose_.transform.rotation.x,
    pose_.transform.rotation.y,
    pose_.transform.rotation.z);
  Eigen::Matrix3d vRt = vQt.toRotationMatrix();
  for (auto light_sample : light_data_) {
    // Lighthouse in the vive frame
    Eigen::Vector3d vPl(
      environment_.lighthouses[light_sample.lighthouse].translation.x,
      environment_.lighthouses[light_sample.lighthouse].translation.y,
      environment_.lighthouses[light_sample.lighthouse].translation.z);
    Eigen::Quaterniond vQl(
      environment_.lighthouses[light_sample.lighthouse].rotation.w,
      environment_.lighthouses[light_sample.lighthouse].rotation.x,
      environment_.lighthouses[light_sample.lighthouse].rotation.y,
      environment_.lighthouses[light_sample.lighthouse].rotation.z);
    Eigen::Matrix3d vRl = vQl.toRotationMatrix();
    // Convert pose to lighthouse frame
    Eigen::Vector3d lPt = vRl.transpose() * (vPt) + ( - vRl.transpose() * vPl);
    Eigen::Matrix3d lRt = vRl.transpose() * vRt;
    for (auto sample : light_sample.samples) {
      Eigen::Vector3d tPs(tracker_.sensors[sample.sensor].position.x,
        tracker_.sensors[sample.sensor].position.y,
        tracker_.sensors[sample.sensor].position.z);
      // Sensor in lighthouse frame
      Eigen::Vector3d lPs = lRt * tPs + lPt;
      // Horizontal Angle
      if (light_sample.axis == HORIZONTAL) {
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_sample.lighthouse].horizontal_motor.phase;
        double tilt = lighthouses_[light_sample.lighthouse].horizontal_motor.tilt;
        double gib_phase = lighthouses_[light_sample.lighthouse].horizontal_motor.gib_phase;
        double gib_mag = lighthouses_[light_sample.lighthouse].horizontal_motor.gib_magnitude;
        double curve = lighthouses_[light_sample.lighthouse].horizontal_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
        } else {
          ang = atan(x);
        }
        // Adding to cost
        cost += pow(sample.angle - ang,2);
        sample_counter++;
      // Vertical Angle
      } else if (light_sample.axis == VERTICAL) {
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_sample.lighthouse].vertical_motor.phase;
        double tilt = lighthouses_[light_sample.lighthouse].vertical_motor.tilt;
        double gib_phase = lighthouses_[light_sample.lighthouse].vertical_motor.gib_phase;
        double gib_mag = lighthouses_[light_sample.lighthouse].vertical_motor.gib_magnitude;
        double curve = lighthouses_[light_sample.lighthouse].vertical_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
        } else {
          ang = atan(y);
        }
        // Adding to cost
        cost += pow(sample.angle - ang,2);
        sample_counter++;
      }
    }
  }

  if (cost > 1e-5 * sample_counter)
    return false;

  return true;
}

void PoseGraph::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  light_data_.push_back(*msg);

  if (!lastposewasimu_) {
    AddPoseBack();
  }
  lastposewasimu_ = false;
  if (light_data_.size() > window_) {
    // Erase first element
    RemoveLight();
  }
  // Add new pose for removal in  IMU cycle
  if (imu_data_.size() != 0 &&
    light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    AddPoseFront();
  }
  // Remove IMU data and poses
  while (imu_data_.size() != 0 &&
    light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    RemoveImu();
  }
  // Solve the problem
  Solve();

  valid_ = Valid();

  return;
}

void PoseGraph::ApplyLimits() {
  // Check if the last measure was imu.
  // If it was then we are covered -- IMU relates two poses
  if (!lastposewasimu_) {
    AddPoseBack();
  }
  // Check the light window limit
  if (light_data_.size() > window_) {
    RemoveLight();
  }
  // Add extra pose to temove later
  if (imu_data_.size() != 0 &&
    light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    AddPoseFront();
  }
  // Remove IMU data and poses
  while (imu_data_.size() != 0 &&
    light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    // Erase out of window IMU msg
    RemoveImu();
  }
  return;
}

void PoseGraph::RemoveLight() {
  light_data_.erase(light_data_.begin());
  poses_.erase(poses_.begin());
  return;
}

void PoseGraph::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // Save the inertial data
  imu_data_.push_back(*msg);
  lastposewasimu_ = true;
  AddPoseBack();

  return;
}

void PoseGraph::RemoveImu() {
  imu_data_.erase(imu_data_.begin());
  poses_.erase(poses_.begin());
  return;
}

void PoseGraph::AddPoseBack() {
  double * new_pose = new double[9];
  if (poses_.size() <= 0) {
    new_pose[0] = 0.0;
    new_pose[1] = 0.0;
    new_pose[2] = 1.0;
    new_pose[3] = 0.0;
    new_pose[4] = 0.0;
    new_pose[5] = 0.0;
    new_pose[6] = 0.0;
    new_pose[7] = 0.0;
    new_pose[8] = 0.0;
  } else {
    new_pose[0] = poses_.back()[0];
    new_pose[1] = poses_.back()[1];
    new_pose[2] = poses_.back()[2];
    new_pose[3] = poses_.back()[3];
    new_pose[4] = poses_.back()[4];
    new_pose[5] = poses_.back()[5];
    new_pose[6] = poses_.back()[6];
    new_pose[7] = poses_.back()[7];
    new_pose[8] = poses_.back()[8];
  }
  poses_.push_back(new_pose);
  return;
}

void PoseGraph::AddPoseFront() {
  double * new_pose = new double[9];
  if (poses_.size() <= 0) {
    new_pose[0] = 0.0;
    new_pose[1] = 0.0;
    new_pose[2] = 1.0;
    new_pose[3] = 0.0;
    new_pose[4] = 0.0;
    new_pose[5] = 0.0;
    new_pose[6] = 0.0;
    new_pose[7] = 0.0;
    new_pose[8] = 0.0;
  } else {
    new_pose[0] = poses_.front()[0];
    new_pose[1] = poses_.front()[1];
    new_pose[2] = poses_.front()[2];
    new_pose[3] = poses_.front()[3];
    new_pose[4] = poses_.front()[4];
    new_pose[5] = poses_.front()[5];
    new_pose[6] = poses_.front()[6];
    new_pose[7] = poses_.front()[7];
    new_pose[8] = poses_.front()[8];
  }
  poses_.insert(poses_.begin(),new_pose);
  return;
}

bool PoseGraph::GetTransform(geometry_msgs::TransformStamped& msg) {
  // Set the output
  msg = pose_;
  // Change the time stamp
  msg.header.stamp == ros::Time::now();
  return true;
}

bool PoseGraph::Solve() {
  // Test if we have enough data
  if (light_data_.size() < window_) return true;
  // Lighthouse and pose
  // std::vector<double*> poses;
  double * prev_pose = NULL;
  DataType prev_type;
  double prev_time = 0;
  double * next_pose = NULL;
  std::string next_lighthouse;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  double * thepose = poses_.back();

  // Iterate over light data
  auto li_it = light_data_.begin();
  auto imu_it = imu_data_.begin();
  auto pose_it = poses_.begin();
  bool lastposewasimu = false;
  size_t next_counter = 0, prev_counter = 0, pose_counter = 0;
  // Preliminary pointer for pre poses
  auto last_pre_pointer = poses_.begin();

  // Preliminary light data - first HORIZONTAL / second VERTICAL
  std::map<std::string, std::pair<hive::ViveLight*,hive::ViveLight*>> pre_data;
  double pre_pose[9];
  pre_pose[0] = poses_.back()[0];
  pre_pose[1] = poses_.back()[1];
  pre_pose[2] = poses_.back()[2];
  pre_pose[3] = poses_.back()[3];
  pre_pose[4] = poses_.back()[4];
  pre_pose[5] = poses_.back()[5];
  pre_pose[6] = poses_.back()[6];
  pre_pose[7] = poses_.back()[7];
  pre_pose[8] = poses_.back()[8];


  // std::cout << "Poses: " << poses_.size() << std::endl;
  while (li_it != light_data_.end()) {
    prev_pose = next_pose;
    next_pose = *pose_it;
    prev_counter = next_counter;
    next_counter = pose_counter;

    // Convert lighthouse to geometry_msgs
    geometry_msgs::Transform lhTF;
    lhTF.translation.x = environment_.lighthouses[li_it->lighthouse].translation.x;
    lhTF.translation.y = environment_.lighthouses[li_it->lighthouse].translation.y;
    lhTF.translation.z = environment_.lighthouses[li_it->lighthouse].translation.z;
    lhTF.rotation.w = environment_.lighthouses[li_it->lighthouse].rotation.w;
    lhTF.rotation.x = environment_.lighthouses[li_it->lighthouse].rotation.x;
    lhTF.rotation.y = environment_.lighthouses[li_it->lighthouse].rotation.y;
    lhTF.rotation.z = environment_.lighthouses[li_it->lighthouse].rotation.z;


    // Iterate IMU data
    while(li_it != light_data_.begin() &&
      imu_it != imu_data_.end() &&
      imu_it->header.stamp < li_it->header.stamp) {
      // Time difference for iteration
      double dt;
      if (prev_type == imu)
        dt = imu_it->header.stamp.toSec() - prev_time;
      else if (prev_type == light)
        dt = 2 * (imu_it->header.stamp.toSec() - prev_time);
      else
        dt = 0.0;
      // Change to the next pose
      // Cost related to inertial measurements
      // std::cout << "InertialCost " << prev_counter << " " << next_counter << std::endl;
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<InertialCost, 7, 9, 9>
        (new InertialCost(*imu_it,
          environment_.gravity,
          tracker_.gyr_bias,
          dt,
          trust_));
      problem.AddResidualBlock(cost, NULL, prev_pose, next_pose);
      pose_it++;
      pose_counter++;
      prev_pose = next_pose;
      next_pose = *pose_it;
      prev_counter = next_counter;
      next_counter = pose_counter;
      lastposewasimu = true;
      // DeltaT
      prev_type = imu;
      prev_time = imu_it->header.stamp.toSec();
      // Next sample
      imu_it++;
    }

    // Go one pose back
    if (lastposewasimu){
      pose_it--;
      pose_counter--;
      prev_pose = *(pose_it-1);
      next_pose = *pose_it;
      prev_counter = pose_counter-1;
      next_counter = pose_counter;
    }

    // Cost related to light measurements
    if (li_it->axis == HORIZONTAL) {
      // std::cout << "ViveHorizontalCost " << next_counter << std::endl;
      ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
        (new ViveHorizontalCost(*li_it,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(9);
      hcost->SetNumResiduals(li_it->samples.size());
      problem.AddResidualBlock(hcost, NULL, next_pose); // replace with next_pose
    } else if (li_it->axis == VERTICAL) {
      // std::cout << "ViveVerticalCost " << next_counter << std::endl;
      ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
        (new ViveVerticalCost(*li_it,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].vertical_motor,
          correction_));
      vcost->AddParameterBlock(9);
      vcost->SetNumResiduals(li_it->samples.size());
      problem.AddResidualBlock(vcost, NULL, next_pose); // replace with next_pose
    }

    // Preliminary solver data
    if (li_it->axis == HORIZONTAL)
      pre_data[li_it->lighthouse].first = &(*li_it);
    else if (li_it->axis == VERTICAL)
      pre_data[li_it->lighthouse].second = &(*li_it);
    // Solve preliminary data
    if (pre_data[li_it->lighthouse].first != NULL
      && pre_data[li_it->lighthouse].second != NULL) {

      pre_pose[0] = 0.0;
      pre_pose[1] = 0.0;
      pre_pose[2] = 1.0;
      pre_pose[3] = 0.0;
      pre_pose[4] = 0.0;
      pre_pose[5] = 0.0;
      pre_pose[6] = 0.0;
      pre_pose[7] = 0.0;
      pre_pose[8] = 0.0;

      ceres::Problem pre_problem;
      ceres::Solver::Options pre_options;
      ceres::Solver::Summary pre_summary;
      // Horizontal data
      ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
        (new ViveHorizontalCost(*pre_data[li_it->lighthouse].first,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(9);
      hcost->SetNumResiduals(pre_data[li_it->lighthouse].first->samples.size());
      pre_problem.AddResidualBlock(hcost, NULL, pre_pose);
      // Vertical data
      ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
        (new ViveVerticalCost(*pre_data[li_it->lighthouse].second,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].vertical_motor,
          correction_));
      vcost->AddParameterBlock(9);
      vcost->SetNumResiduals(pre_data[li_it->lighthouse].second->samples.size());
      pre_problem.AddResidualBlock(vcost, NULL, pre_pose);
      pre_options.minimizer_progress_to_stdout = false;
      pre_options.max_num_iterations = 1000;
      ceres::Solve(pre_options, &pre_problem, &pre_summary);
      // Copy paste

      auto pointer_pose = last_pre_pointer;
      while (pointer_pose != pose_it + 1) {
        for (size_t i = 0; i < 9; i++) {
          (*pointer_pose)[i] = pre_pose[i];
        }
        pointer_pose++;
      }
      last_pre_pointer = pose_it + 1;
    }

    // Next pose
    pose_it++;
    pose_counter++;
    lastposewasimu = false;

    // Delta time
    prev_type = light;
    prev_time = li_it->header.stamp.toSec();

    // Next sample
    li_it++;
  }

  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  // size_t counter = 0;
  // for (auto pose : poses_) {
  //   std::cout << counter << " "
  //     << summary.final_cost <<  " - "
  //     << pose[0] << ", "
  //     << pose[1] << ", "
  //     << pose[2] << ", "
  //     << pose[3] << ", "
  //     << pose[4] << ", "
  //     << pose[5] << ", "
  //     << pose[6] << ", "
  //     << pose[7] << ", "
  //     << pose[8] << std::endl;
  //     counter++;
  // }
  std::cout << summary.final_cost <<  " - "
    << poses_.back()[0] << ", "
    << poses_.back()[1] << ", "
    << poses_.back()[2] << ", "
    << poses_.back()[6] << ", "
    << poses_.back()[7] << ", "
    << poses_.back()[8] << std::endl;

  // Save pose -- light frame in the vive frame
  pose_.header.stamp = light_data_.back().header.stamp;
  pose_.header.frame_id = "vive";
  pose_.child_frame_id = tracker_.serial;
  // The computed pose
  Eigen::Vector3d vPi(poses_.back()[0],
    poses_.back()[1],
    poses_.back()[2]);
  Eigen::Vector3d vAi(poses_.back()[6],
    poses_.back()[7],
    poses_.back()[8]);
  Eigen::AngleAxisd vAAi(vAi.norm(), vAi.normalized());
  Eigen::Matrix3d vRi = vAAi.toRotationMatrix();
  // Imu to light
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();
  // Convert frames
  Eigen::Vector3d vPt = vRi * ( - tRi.transpose() * tPi ) + tPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();
  Eigen::Quaterniond vQt(vRt);
  // Save to ROS msg
  pose_.transform.translation.x = vPt(0);
  pose_.transform.translation.y = vPt(1);
  pose_.transform.translation.z = vPt(2);
  pose_.transform.rotation.w = vQt.w();
  pose_.transform.rotation.x = vQt.x();
  pose_.transform.rotation.y = vQt.y();
  pose_.transform.rotation.z = vQt.z();

  return true;
}

void PoseGraph::PrintState() {
  return;
}


int main(int argc, char ** argv) {
  // ros intializations
  std::map<std::string, PoseGraph> smap;
  Calibration cal;
  rosbag::View view;
  rosbag::Bag rbag;

  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << std::endl;
    return -1;
  }

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
    ROS_FATAL("Can't find calibration file.");
    return -1;
  } else {
    ROS_INFO("Read calibration file.");
  }

  rbag.open(argv[1], rosbag::bagmode::Read);
  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal.SetTrackers(*vt);
    for (auto tr : vt->trackers) {
      smap[tr.serial] = PoseGraph(cal.environment,
        cal.trackers[tr.serial],
        cal.lighthouses,
        4, TRUST, true);
    }
  }
  ROS_INFO("Trackers' setup complete.");

  size_t counter = 0;
  // Light data
  std::vector<std::string> run_topics; 
  run_topics.push_back("/loc/vive/light");
  run_topics.push_back("/loc/vive/imu/");
  run_topics.push_back("/loc/vive/imu");
  rosbag::View view_li(rbag, rosbag::TopicQuery(run_topics));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    if (vl != NULL) {
      // std::cout << "LIGHT" << std::endl;
      smap[vl->header.frame_id].ProcessLight(vl);
      counter++;
    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      // std::cout << "IMU" << std::endl;
      smap[vi->header.frame_id].ProcessImu(vi);
      counter++;
    }
    // if (counter >= 200) break;
  }
  ROS_INFO("Data processment complete.");

  return 0;
}