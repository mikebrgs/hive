#include <hive/vive_refine.h>

#define ROTATION_COST_FACTOR 1.0

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;
typedef std::map<std::string, std::pair<hive::ViveLight*,
  hive::ViveLight*>> LightMap;


Refinery::Refinery(Calibration & calibration) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = true;
  inertial_ = false;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  inertial_ = false;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction,
  double smoothing) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = smoothing;
  inertial_ = false;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction,
  double smoothing,
  bool inertial) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = smoothing;
  inertial_ = inertial;
  return;
}

Refinery::~Refinery() {
  // pass
}

bool Refinery::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  return true; // nothing for now
  if (msg == NULL) return false;

  data_[msg->header.frame_id].second.push_back(*msg);

  return true;
}

bool Refinery::AddLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return false;

  data_[msg->header.frame_id].first.push_back(*msg);

  return true;
}

bool Refinery::Solve() {
  if (inertial_)
    return SolveInertial();
  return SolveStatic();
}

typedef std::map<std::string, std::pair<hive::ViveLight*,hive::ViveLight*>> LightPointerMapPair;
typedef std::map<std::string, std::vector<double*>> PoseVectorMap;
typedef std::map<std::string, double*> PoseMap;

bool Refinery::SolveInertial() {
  // All the poses
  PoseVectorMap poses;
  PoseMap lighthouses;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Initialize lighthouses
  for (auto lighthouse : lighthouses) {
    // Position
    lighthouse.second[0] =
      calibration_.environment.lighthouses[lighthouse.first].translation.x;
    lighthouse.second[1] =
      calibration_.environment.lighthouses[lighthouse.first].translation.y;
    lighthouse.second[2] =
      calibration_.environment.lighthouses[lighthouse.first].translation.z;
    // Orientation
    Eigen::Quaterniond vQl(
      calibration_.environment.lighthouses[lighthouse.first].rotation.w,
      calibration_.environment.lighthouses[lighthouse.first].rotation.x,
      calibration_.environment.lighthouses[lighthouse.first].rotation.y,
      calibration_.environment.lighthouses[lighthouse.first].rotation.z);
    Eigen::AngleAxisd vAAl(vQl);
    lighthouse.second[3] = vAAl.angle() * vAAl.axis()(0);
    lighthouse.second[4] = vAAl.angle() * vAAl.axis()(1);
    lighthouse.second[5] = vAAl.angle() * vAAl.axis()(2);
  }

  for (auto tracker_data : data_) {
    // More readable structures
    SweepVec light_data = tracker_data.second.first;
    ImuVec imu_data = tracker_data.second.second;
    Tracker tracker = calibration_.trackers[tracker_data.first];

    // Other initializations
    bool lastposewasimu = false;
    ros::Time prev_time;

    // Auxiliary structures
    LightPointerMapPair pre_data;

    // Pose pointers
    double * next_pose = nullptr;
    double * prev_pose = nullptr;

    // Initialize iterators
    auto li_it = light_data.begin();
    auto imu_it = imu_data.begin();
    auto pose_it = poses[tracker.serial].begin();
    // Iterate light data
    while (li_it != light_data.end()) {

      // Iterate imu data
      while (imu_it != imu_data.end()
        && imu_it->header.stamp < li_it->header.stamp
        && li_it != light_data.begin()) {
        // New pose for imu data
        poses[tracker.serial].push_back(new double[9]);
        prev_pose = next_pose;
        next_pose = poses[tracker.serial].back();
        lastposewasimu = true;
        // Time delta
        double dt = (imu_it->header.stamp - prev_time).toSec();
        if (!lastposewasimu)
          dt = 2*dt;
        // Cost related to inertial measurements
        // std::cout << "InertialCost " << prev_counter << " " << next_counter << std::endl;
        ceres::CostFunction * cost =
          new ceres::AutoDiffCostFunction<InertialCost, 7, 9, 9>
          (new InertialCost(*imu_it,
            calibration_.environment.gravity,
            tracker.gyr_bias,
            dt,
            smoothing_));
        problem.AddResidualBlock(cost, NULL, prev_pose, next_pose);

        prev_time = imu_it->header.stamp;
      }

      // New Light pose
      if (!lastposewasimu) {
        // Will be initialized later on
        poses[tracker.serial].push_back(new double[9]);
        prev_pose = next_pose;
        next_pose = poses[tracker.serial].back();
        lastposewasimu = false;
      }
      // Cost related to light measurements
      if (li_it->axis == HORIZONTAL) {
        // std::cout << "ViveCalibrationHorizontalCost " << next_counter << std::endl;
        ceres::DynamicAutoDiffCostFunction<ViveCalibrationHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<ViveCalibrationHorizontalCost, 4>
          (new ViveCalibrationHorizontalCost(*li_it,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(9);
        hcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(hcost, NULL, next_pose); // replace with next_pose
      } else if (li_it->axis == VERTICAL) {
        // std::cout << "ViveCalibrationVerticalCost " << next_counter << std::endl;
        ceres::DynamicAutoDiffCostFunction<ViveCalibrationVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<ViveCalibrationVerticalCost, 4>
          (new ViveCalibrationVerticalCost(*li_it,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(9);
        vcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(vcost, NULL, next_pose); // replace with next_pose
      }
      prev_time = li_it->header.stamp;

      // Save data
      if (li_it->axis == HORIZONTAL)
        pre_data[li_it->lighthouse].first = &(*li_it);
      else if (li_it->axis == VERTICAL)
        pre_data[li_it->lighthouse].second = &(*li_it);
      // Initialize poses for solver
      if (pre_data[li_it->lighthouse].first != NULL
        && pre_data[li_it->lighthouse].second != NULL) {

        double pre_pose[9];
        pre_pose[0] = 0.0;
        pre_pose[1] = 0.0;
        pre_pose[2] = 1.0;
        pre_pose[3] = 0.0;
        pre_pose[4] = 0.0;
        pre_pose[5] = 0.0;
        pre_pose[6] = 0.0;
        pre_pose[7] = 0.0;
        pre_pose[8] = 0.0;

        // Lighthouse
        geometry_msgs::Transform lighthouse;
        lighthouse.translation =
          calibration_.environment.lighthouses[li_it->lighthouse].translation;
        lighthouse.rotation =
          calibration_.environment.lighthouses[li_it->lighthouse].rotation;

        ceres::Problem pre_problem;
        ceres::Solver::Options pre_options;
        ceres::Solver::Summary pre_summary;
        // Horizontal data
        ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
          (new ViveHorizontalCost(*pre_data[li_it->lighthouse].first,
            lighthouse,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(9);
        hcost->SetNumResiduals(pre_data[li_it->lighthouse].first->samples.size());
        pre_problem.AddResidualBlock(hcost, NULL, pre_pose);
        // Vertical data
        ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
          (new ViveVerticalCost(*pre_data[li_it->lighthouse].second,
            lighthouse,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(9);
        vcost->SetNumResiduals(pre_data[li_it->lighthouse].second->samples.size());
        pre_problem.AddResidualBlock(vcost, NULL, pre_pose);

        // Solve
        pre_options.minimizer_progress_to_stdout = false;
        pre_options.max_num_iterations = 1000;
        ceres::Solve(pre_options, &pre_problem, &pre_summary);

        // Copy paste
        while (pose_it != poses[tracker.serial].end()) {
          for (size_t i = 0; i < 9; i++)
            (*pose_it)[i] = pre_pose[i];
          pose_it++;
        }
        // End of Copy Past
      }
      // End of Initializer
    }
    // End of light_it
  }
  // End of tracker_data

  // Fix one of the lighthouses to the vive frame
  problem.SetParameterBlockConstant(lighthouses.begin()->second);

  for (auto lh_it = lighthouses.begin(); lh_it != lighthouses.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << std::endl;
  // Solver's options
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000; // TODO change this

  ceres::Solve(options, &problem, &summary);

  for (auto lh_it = lighthouses.begin(); lh_it != lighthouses.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << std::endl;

  std::cout << summary.BriefReport() << std::endl;

  // Save all the new lighthouse poses
  for (auto lighthouse : lighthouses) {
    calibration_.environment.lighthouses[lighthouse.first].translation.x
      = lighthouse.second[0];
    calibration_.environment.lighthouses[lighthouse.first].translation.y
      = lighthouse.second[1];
    calibration_.environment.lighthouses[lighthouse.first].translation.z
      = lighthouse.second[2];
    Eigen::Vector3d vAl(lighthouse.second[3],
      lighthouse.second[4],
      lighthouse.second[5]);
    Eigen::AngleAxisd vAAl;
    if (vAl.norm() != 0)
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl.normalized());
    else 
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl);
    Eigen::Quaterniond vQl(vAAl);
    calibration_.environment.lighthouses[lighthouse.first].rotation.w
      = vQl.w();
    calibration_.environment.lighthouses[lighthouse.first].rotation.x
      = vQl.x();
    calibration_.environment.lighthouses[lighthouse.first].rotation.y
      = vQl.y();
    calibration_.environment.lighthouses[lighthouse.first].rotation.z
      = vQl.z();
  }

  return true;
}

// Solve the problem
bool Refinery::SolveStatic() {
  // Check requirements
  if (calibration_.environment.lighthouses.size() <= 1) {
    ROS_WARN("Not enough lighthouses for Refinement.");
    return false;
  }

  // Ceres set up
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;


  // Environment transforms
  std::map<std::string, double[6]> vTl;
  for (auto lh_it = calibration_.environment.lighthouses.begin();
    lh_it != calibration_.environment.lighthouses.end(); lh_it++) {
    std::cout << "LH: " << lh_it->first << std::endl;
    // Conversion from quaternion to angle axis
    Eigen::AngleAxisd vAl(Eigen::Quaterniond(
      lh_it->second.rotation.w,
      lh_it->second.rotation.x,
      lh_it->second.rotation.y,
      lh_it->second.rotation.z));
    // Conversion to double array
    vTl[lh_it->first][0] = lh_it->second.translation.x;
    vTl[lh_it->first][1] = lh_it->second.translation.y;
    vTl[lh_it->first][2] = lh_it->second.translation.z;
    vTl[lh_it->first][3] = vAl.axis()(0) * vAl.angle();
    vTl[lh_it->first][4] = vAl.axis()(1) * vAl.angle();
    vTl[lh_it->first][5] = vAl.axis()(2) * vAl.angle();
  }

  double * prev_pose = NULL;
  double * next_pose = NULL;
  std::string prev, next;
  // first -- horizontal observations
  // second -- vertical observations
  LightMap observations;
  // Vector to save the poses
  std::vector<double*> poses;

  // Iterate tracker data
  for (auto tr_it = data_.begin(); tr_it != data_.end(); tr_it++) {
    // Iterate light data
    for (auto li_it = tr_it->second.first.begin();
      li_it != tr_it->second.first.end(); li_it++) {
      // Check if lighthouse is in calibration -- if not continue
      if (vTl.find(li_it->lighthouse) == vTl.end()) {
        continue;
      }
      // Initialize observation for lighthouse
      if (observations.find(li_it->lighthouse) == observations.end()) {
        observations[li_it->lighthouse].first = NULL; // horizontal
        observations[li_it->lighthouse].second = NULL; // vertical
      }
      // Compute first individual poses
      if (li_it->axis == HORIZONTAL) {
        // Convert from iterator to pointer
        observations[li_it->lighthouse].first = &(*li_it);
      } else if (li_it->axis == VERTICAL) {
        // Convert from iterator to pointer
        observations[li_it->lighthouse].second  = &(*li_it);
      }
      TF tf;
      if (observations[li_it->lighthouse].first != NULL &&
        observations[li_it->lighthouse].second != NULL &&
        ViveSolve::SolvePose(*observations[li_it->lighthouse].first,
        *observations[li_it->lighthouse].second,
        tf, calibration_.trackers[tr_it->first],
        calibration_.lighthouses[li_it->lighthouse],
        correction_)) {
        double * pose = new double[6];
        // Set new pose from solver
        pose[0] = tf.transform.translation.x;
        pose[1] = tf.transform.translation.y;
        pose[2] = tf.transform.translation.z;
        Eigen::Quaterniond Q(tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);
        Eigen::AngleAxisd A(Q);
        pose[3] = A.axis()(0) * A.angle();
        pose[4] = A.axis()(1) * A.angle();
        pose[5] = A.axis()(2) * A.angle();
        // Consecutive pose memory
        prev = next;
        next = li_it->lighthouse;
        prev_pose = next_pose;
        next_pose = pose;
        // Horizontal cost
        if (li_it->axis == HORIZONTAL) {
          ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4> * hcost =
            new ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4>
            (new PoseHorizontalCost(*observations[li_it->lighthouse].first,
              calibration_.trackers[tr_it->first],
              calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
              correction_));
          hcost->AddParameterBlock(6);
          hcost->SetNumResiduals(observations[li_it->lighthouse].first->samples.size());
          problem.AddResidualBlock(hcost, new ceres::HuberLoss(0.05), next_pose);
        // Vertical cost
        } else if (li_it->axis == VERTICAL) {
          ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4> * vcost =
            new ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4>
            (new PoseVerticalCost(*observations[li_it->lighthouse].second,
              calibration_.trackers[tr_it->first],
              calibration_.lighthouses[li_it->lighthouse].vertical_motor,
              correction_));
          vcost->AddParameterBlock(6);
          vcost->SetNumResiduals(observations[li_it->lighthouse].second->samples.size());
          problem.AddResidualBlock(vcost, new ceres::HuberLoss(0.05), next_pose);
        }
        poses.push_back(pose);
        // Smoothing cost
        // If the poses are in the same frame
        if (next == prev && prev_pose != NULL && next_pose != NULL) {
          ceres::CostFunction * cost =
            new ceres::AutoDiffCostFunction<SmoothingCost, 4, 6, 6, 6>
            (new SmoothingCost(smoothing_, ROTATION_COST_FACTOR));
          problem.AddResidualBlock(cost, new ceres::HuberLoss(0.01),
            prev_pose, next_pose, vTl[li_it->lighthouse]);
        // If the poses are in different frames
        } else if (prev_pose != NULL && next_pose != NULL) {
          ceres::CostFunction * cost =
            new ceres::AutoDiffCostFunction<SmoothingCost, 4, 6, 6, 6, 6>
            (new SmoothingCost(smoothing_, ROTATION_COST_FACTOR));
          problem.AddResidualBlock(cost, new ceres::HuberLoss(0.01),
            prev_pose, vTl[prev], next_pose, vTl[next]);
        }
      }
    }
  }

  // Fix one of the lighthouses to the vive frame
  problem.SetParameterBlockConstant(vTl.begin()->second);

  for (auto lh_it = vTl.begin(); lh_it != vTl.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << std::endl;
  // Solver's options
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000; // TODO change this

  ceres::Solve(options, &problem, &summary);

  for (auto lh_it = vTl.begin(); lh_it != vTl.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << std::endl;

  std::cout << summary.BriefReport() << std::endl;

  // Save all the new lighthouse poses
  for (auto lighthouse : vTl) {
    calibration_.environment.lighthouses[lighthouse.first].translation.x
      = lighthouse.second[0];
    calibration_.environment.lighthouses[lighthouse.first].translation.y
      = lighthouse.second[1];
    calibration_.environment.lighthouses[lighthouse.first].translation.z
      = lighthouse.second[2];
    Eigen::Vector3d vAl(lighthouse.second[3],
      lighthouse.second[4],
      lighthouse.second[5]);
    Eigen::AngleAxisd vAAl;
    if (vAl.norm() != 0)
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl.normalized());
    else 
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl);
    Eigen::Quaterniond vQl(vAAl);
    calibration_.environment.lighthouses[lighthouse.first].rotation.w
      = vQl.w();
    calibration_.environment.lighthouses[lighthouse.first].rotation.x
      = vQl.x();
    calibration_.environment.lighthouses[lighthouse.first].rotation.y
      = vQl.y();
    calibration_.environment.lighthouses[lighthouse.first].rotation.z
      = vQl.z();
  }

  return true;
}

Calibration Refinery::GetCalibration() {
  return calibration_;
}

// This is a test function
int main(int argc, char ** argv)
{
  // ros intializations
  rosbag::View view;
  rosbag::Bag rbag;

  // Refinery initializations
  Calibration cal;

  // Read the bag name
  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << "name_of_write_bag.bag" << std::endl;
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
  }
  ROS_INFO("Trackers' setup complete.");

  size_t counter = 0;
  Refinery ref = Refinery(cal, true, 1e1);
  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    ref.AddLight(vl);
    counter++;
    if (counter >= 100) break;
  }
  ROS_INFO("Data processment complete.");

  // Solve the refinement
  ref.Solve();

  ViveUtils::WriteConfig(HIVE_CALIBRATION_FILE,
    ref.GetCalibration());

  return 0;
}