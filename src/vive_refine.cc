#include <hive/vive_refine.h>

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;
typedef std::map<std::string, std::pair<hive::ViveLight*,
  hive::ViveLight*>> LightMap;

// Constructor to pass data
SmoothingCost::SmoothingCost(double smoothing) {
  smoothing_ = smoothing;
}
// Function for ceres solver with parameters
template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
  const T* const prev_vTl,
  const T* const next_lTt,
  const T* const next_vTl,
  T * residual) const {
  // cost function here

  // Frame conversion
  Eigen::Matrix<T, 3, 1> prev_lPt;
  prev_lPt << prev_lTt[0],
    prev_lTt[1],
    prev_lTt[2];
  Eigen::Matrix<T, 3, 1> next_lPt;
  next_lPt << next_lTt[0],
    next_lTt[1],
    next_lTt[2];
  Eigen::Matrix<T, 3, 3> prev_lRt;
  ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
  Eigen::Matrix<T, 3, 3> next_lRt;
  ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

  Eigen::Matrix<T, 3, 1> prev_vPl;
  prev_vPl << prev_vTl[0],
    prev_vTl[1],
    prev_vTl[2];
  Eigen::Matrix<T, 3, 1> next_vPl;
  next_vPl << next_vTl[0],
    next_vTl[1],
    next_vTl[2];
  Eigen::Matrix<T, 3, 3> prev_vRl;
  ceres::AngleAxisToRotationMatrix(&prev_vTl[3], prev_vRl.data());
  Eigen::Matrix<T, 3, 3> next_vRl;
  ceres::AngleAxisToRotationMatrix(&next_vTl[3], next_vRl.data());

  Eigen::Matrix<T, 3, 1> prev_vPt = prev_vRl * prev_lPt + prev_vPl;
  Eigen::Matrix<T, 3, 3> prev_vRt = prev_vRl * prev_lRt;

  Eigen::Matrix<T, 3, 1> next_vPt = next_vRl * next_lPt + next_vPl;
  Eigen::Matrix<T, 3, 3> next_vRt = next_vRl * next_lRt;

  // // Translation cost with smoothing
  residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
  residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
  residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
  // // Rotation cost with smoothing
  T aa[3];
  Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
  ceres::RotationMatrixToAngleAxis(R.data(), aa);
  residual[3] = T(smoothing_) * sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
  return true;
}

template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
  const T* const next_lTt,
  const T* const vTl,
  T * residual) const {
  // Frame conversion
  Eigen::Matrix<T, 3, 1> prev_lPt;
  prev_lPt << prev_lTt[0],
    prev_lTt[1],
    prev_lTt[2];
  Eigen::Matrix<T, 3, 1> next_lPt;
  next_lPt << next_lTt[0],
    next_lTt[1],
    next_lTt[2];
  Eigen::Matrix<T, 3, 3> prev_lRt;
  ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
  Eigen::Matrix<T, 3, 3> next_lRt;
  ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

  Eigen::Matrix<T, 3, 1> vPl;
  vPl << vTl[0],
    vTl[1],
    vTl[2];
  Eigen::Matrix<T, 3, 3> vRl;
  ceres::AngleAxisToRotationMatrix(&vTl[3], vRl.data());

  Eigen::Matrix<T, 3, 1> prev_vPt = vRl * prev_lPt + vPl;
  Eigen::Matrix<T, 3, 3> prev_vRt = vRl * prev_lRt;

  Eigen::Matrix<T, 3, 1> next_vPt = vRl * next_lPt + vPl;
  Eigen::Matrix<T, 3, 3> next_vRt = vRl * next_lRt;

  // // Translation cost with smoothing
  residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
  residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
  residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
  // // Rotation cost with smoothing
  T aa[3];
  Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
  ceres::RotationMatrixToAngleAxis(R.data(), aa);
  residual[3] = T(smoothing_) * sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
  return true;
}


Refinery::Refinery(Calibration & calibration) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = true;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration, bool correction) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration, bool correction, double smoothing) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = smoothing;
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

// Solve the problem
bool Refinery::Solve() {
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
          problem.AddResidualBlock(hcost, NULL, next_pose);
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
          problem.AddResidualBlock(vcost, NULL, next_pose);
        }
        poses.push_back(pose);
        // Smoothing cost
        // If the poses are in the same frame
        if (next == prev && prev_pose != NULL && next_pose != NULL) {
          ceres::CostFunction * cost =
            new ceres::AutoDiffCostFunction<SmoothingCost, 4, 6, 6, 6>
            (new SmoothingCost(smoothing_));
          problem.AddResidualBlock(cost, NULL,
            prev_pose, next_pose, vTl[li_it->lighthouse]);
        // If the poses are in different frames
        } else if (prev_pose != NULL && next_pose != NULL) {
          ceres::CostFunction * cost =
            new ceres::AutoDiffCostFunction<SmoothingCost, 4, 6, 6, 6, 6>
            (new SmoothingCost(smoothing_));
          problem.AddResidualBlock(cost, NULL,
            prev_pose, vTl[prev], next_pose, vTl[next]);
        }
      }
    }
  }

  for (auto po_it = poses.begin(); po_it != poses.end(); po_it++) {
    std::cout << (*po_it)[0] << ", "
      << (*po_it)[1] << ", "
      << (*po_it)[2] << ", "
      << (*po_it)[3] << ", "
      << (*po_it)[4] << ", "
      << (*po_it)[5] << std::endl;
  }
  std::cout << std::endl;
  // Solver's options
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;

  ceres::Solve(options, &problem, &summary);

  for (auto po_it = poses.begin(); po_it != poses.end(); po_it++) {
    std::cout << (*po_it)[0] << ", "
      << (*po_it)[1] << ", "
      << (*po_it)[2] << ", "
      << (*po_it)[3] << ", "
      << (*po_it)[4] << ", "
      << (*po_it)[5] << std::endl;
  }
  std::cout << std::endl;

  std::cout << summary.BriefReport() << std::endl;

  return true;
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
  Refinery ref = Refinery(cal, false, 1e-1);
  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    ref.AddLight(vl);
    counter++;
    if (counter >= 10) break;
  }
  ROS_INFO("Data processment complete.");

  // Solve the refinement
  ref.Solve();

  return 0;
}