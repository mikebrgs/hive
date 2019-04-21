#include <hive/vive_pgo.h>

#define DT 0.005
#define TRUST 1

ViveHorizontalCost::ViveHorizontalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
  return;
}

ViveHorizontalCost::~ViveHorizontalCost() {
  // Do nothing
  return;
}

template <typename T>
bool ViveHorizontalCost::operator()(const T* const * parameters,
  T* residual) const {
  // Optimization parameters
  Eigen::Matrix<T, 3, 1> vPt;
  vPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());

  // Lighthouse pose
  Eigen::Matrix<T, 3, 1> vPl;
  vPl << T(lTv_.translation.x),
    T(lTv_.translation.y),
    T(lTv_.translation.z);
  Eigen::Quaternion<T> vQl(
    T(lTv_.rotation.w),
    T(lTv_.rotation.x),
    T(lTv_.rotation.y),
    T(lTv_.rotation.z));
  Eigen::Matrix<T, 3, 3> vRl;
  vRl = vQl.toRotationMatrix();

  // Vive frame to lighthouse
  Eigen::Matrix<T, 3, 1> lPv;
  lPv = - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRv;
  lRv = vRl.transpose();
  Eigen::Matrix<T, 3, 1> lPt;
  lPt = lRv * vPt + lPv;
  Eigen::Matrix<T, 3, 3> lRt;
  lRt = lRv * vRt;

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

ViveVerticalCost::ViveVerticalCost(hive::ViveLight data,
  geometry_msgs::Transform lTv,
  Tracker tracker,
  Motor lighthouse,
  bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
  return;
}

ViveVerticalCost::~ViveVerticalCost() {
  // Do nothing
  return;
}

template <typename T>
bool ViveVerticalCost::operator()(const T* const * parameters,
  T* residual) const {
  // Optimization parameters
  Eigen::Matrix<T, 3, 1> vPt;
  vPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());

  // Lighthouse pose
  Eigen::Matrix<T, 3, 1> vPl;
  vPl << T(lTv_.translation.x),
    T(lTv_.translation.y),
    T(lTv_.translation.z);
  Eigen::Quaternion<T> vQl(
    T(lTv_.rotation.w),
    T(lTv_.rotation.x),
    T(lTv_.rotation.y),
    T(lTv_.rotation.z));
  Eigen::Matrix<T, 3, 3> vRl;
  vRl = vQl.toRotationMatrix();

  // Vive frame to lighthouse
  Eigen::Matrix<T, 3, 1> lPv;
  lPv = - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRv;
  lRv = vRl.transpose();
  Eigen::Matrix<T, 3, 1> lPt;
  lPt = lRv * vPt + lPv;
  Eigen::Matrix<T, 3, 3> lRt;
  lRt = lRv * vRt;

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


InertialCost::InertialCost(sensor_msgs::Imu imu,
  geometry_msgs::Transform imu_T, // from imu frame to light frame
  geometry_msgs::Vector3 gravity,
  double time_step,
  double trust_weight) {
  imu_ = imu;
  imu_T_ = imu_T;
  gravity_ = gravity;
  time_step_ = time_step;
  trust_weight_ = trust_weight;
  return;
}

InertialCost::~InertialCost() {
  // Nothing happens
  return;
}

template <typename T>
bool InertialCost::operator()(const T* const prev_vTt,
  const T* const next_vTt,
  T * residual) const {

  // prev_vTt's state
  Eigen::Matrix<T,3,1> prev_vPt;
  prev_vPt << prev_vTt[0],
    prev_vTt[1],
    prev_vTt[2];
  Eigen::Matrix<T,3,1> prev_vVt;
  prev_vVt << prev_vTt[3],
    prev_vTt[4],
    prev_vTt[5];
  T paux_vQt[3];
  ceres::AngleAxisToQuaternion(&prev_vTt[6], paux_vQt);
  Eigen::Quaternion<T> prev_vQt(paux_vQt[0],
    paux_vQt[1],
    paux_vQt[2],
    paux_vQt[3]);
  Eigen::Matrix<T,3,3> prev_vRt = prev_vQt.toRotationMatrix();

  // next_vTt's state
  Eigen::Matrix<T,3,1> next_vPt;
  next_vPt << next_vTt[0],
    next_vTt[1],
    next_vTt[2];
  Eigen::Matrix<T,3,1> next_vVt;
  next_vVt << next_vTt[3],
    next_vTt[4],
    next_vTt[5];
  T naux_vQt[3];
  ceres::AngleAxisToQuaternion(&next_vTt[6], naux_vQt);
  Eigen::Quaternion<T> next_vQt(paux_vQt[0],
    paux_vQt[1],
    paux_vQt[2],
    paux_vQt[3]);
  Eigen::Matrix<T,3,3> next_vRt = next_vQt.toRotationMatrix();

  // Tracker's imu frame
  Eigen::Matrix<T,3,1> tPi;
  tPi << T(imu_T_.translation.x),
    T(imu_T_.translation.y),
    T(imu_T_.translation.z);
  Eigen::Matrix<T,3,3> tRi;
  tRi = Eigen::Quaternion<T>(T(imu_T_.rotation.w),
    T(imu_T_.rotation.x),
    T(imu_T_.rotation.y),
    T(imu_T_.rotation.z)).toRotationMatrix();

  // Inertial measurements
  Eigen::Matrix<T,3,1> iW;
  iW << T(imu_.angular_velocity.x),
    T(imu_.angular_velocity.y),
    T(imu_.angular_velocity.z);
  Eigen::Matrix<T,3,1> iA;
  iA << T(imu_.linear_acceleration.x),
    T(imu_.linear_acceleration.y),
    T(imu_.linear_acceleration.z);

  // Gravity
  Eigen::Matrix<T,3,1> vG;
  vG << T(gravity_.x),
    T(gravity_.y),
    T(gravity_.z);

  // Inertial predictions
  Eigen::Matrix<T,3,1> est_vPt;
  est_vPt = prev_vPt + (T(time_step_) * prev_vVt);
  Eigen::Matrix<T,3,1> est_vVt;
  est_vVt = prev_vVt + (T(time_step_) * prev_vRt * (tRi * 
    iA) + vG);
  Eigen::Matrix<T,4,3> Omega;
  Omega << -prev_vQt.x(), -prev_vQt.y(), -prev_vQt.z(),
    prev_vQt.w(), -prev_vQt.z(), prev_vQt.y(),
    prev_vQt.z(), prev_vQt.w(), -prev_vQt.x(),
    -prev_vQt.y(), prev_vQt.x(), prev_vQt.w();
  Eigen::Matrix<T,4,1> trev_vQt; // temporary previous
  trev_vQt << prev_vQt.w(),
    prev_vQt.x(),
    prev_vQt.y(),
    prev_vQt.z();
  Eigen::Matrix<T,4,1> text_vQt; // temporary next
  text_vQt = trev_vQt + T(time_step_) * T(0.5) * Omega * tRi * iW; // Wrong -- change the tracking to the imu
  // this way we would have to had the influence of w to the linear velocity
  Eigen::Quaternion<T> est_vQt(text_vQt(0),
    text_vQt(1),
    text_vQt(2),
    text_vQt(3));
  est_vQt.normalize();
  Eigen::Matrix<T,3,3> est_vRt = est_vQt.toRotationMatrix();

  Eigen::Matrix<T,3,1> dP;
  Eigen::Matrix<T,3,1> dV;
  Eigen::Matrix<T,3,3> dR;
  dP = next_vPt - est_vPt;
  dV = next_vVt - est_vVt;
  dR = next_vRt.transpose() * prev_vRt;

  // Position cost
  residual[0] = T(trust_weight_) * dP(0);
  residual[1] = T(trust_weight_) * dP(1);
  residual[2] = T(trust_weight_) * dP(2);
  // Velocity cost
  residual[0] = T(trust_weight_) * dV(0);
  residual[1] = T(trust_weight_) * dV(1);
  residual[2] = T(trust_weight_) * dV(2);
  // Orientation cost
  T aa[3];
  ceres::RotationMatrixToAngleAxis(dR.data(), aa);
  residual[3] = T(trust_weight_) *
    sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);

  return true;
}

PoseGraph::PoseGraph(Environment environment,
  Tracker tracker,
  std::map<std::string, Lighthouse> lighthouses,
  size_t window,
  bool correction) {
  if (window < 2) {
    std::cout << "Bad window size. Using 2." << std::endl;
    window_ = 2;
  } else {
    window_ = window;
  }
  valid_ = false;
  correction_ = correction;
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

void PoseGraph::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  light_data_.push_back(*msg);
  poses_.push_back(new double[6]);
  ProcessPose();
  if (light_data_.size() > window_) {
    // Erase first element
    RemoveLight();
  }
  while (imu_data_.size() != 0 &&
    light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    // Erase out of window IMU msg
    RemoveImu();
  }
  // Solve the problem
  Solve();
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
  poses_.push_back(new double[6]);
  ProcessPose();
  return;
}

void PoseGraph::ProcessPose() {
  if (poses_.size() == 1) {
    poses_.back()[0] = 0.0;
    poses_.back()[1] = 0.0;
    poses_.back()[2] = 1.0;
    poses_.back()[3] = 0.0;
    poses_.back()[4] = 0.0;
    poses_.back()[5] = 0.0;
  } else {
    poses_.back()[0] = poses_.front()[0];
    poses_.back()[1] = poses_.front()[1];
    poses_.back()[2] = poses_.front()[2];
    poses_.back()[3] = poses_.front()[3];
    poses_.back()[4] = poses_.front()[4];
    poses_.back()[5] = poses_.front()[5];
  }
  return;
}

void PoseGraph::RemoveImu() {
  imu_data_.erase(imu_data_.begin());
  poses_.erase(poses_.begin());
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
  // Lighthouse and pose
  // std::vector<double*> poses;
  double * prev_pose = NULL;
  std::string prev_lighthouse;
  double * next_pose = NULL;
  std::string next_lighthouse;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // double * thepose = new double[6];
  poses_.back()[0] = 0.0;
  poses_.back()[1] = 0.0;
  poses_.back()[2] = 1.0;
  poses_.back()[3] = 0.0;
  poses_.back()[4] = 0.0;
  poses_.back()[5] = 0.0;

  // Iterate over light data
  auto li_it = light_data_.begin();
  auto imu_it = imu_data_.begin();
  auto pose_it = poses_.begin();
  while (li_it != light_data_.end()) {
    // std::cout << li_it->header.stamp
    //   << " - " << int(li_it->axis) << std::endl;
    prev_pose = next_pose;
    // prev_lighthouse = next_lighthouse;
    next_pose = *pose_it;
    // next_lighthouse = li_it->lighthouse;
    // TOOD initialize pose
    // Convert lighthouse to geometry_msgs
    geometry_msgs::Transform lhTF;
    lhTF.translation.x = environment_.lighthouses[li_it->lighthouse].translation.x;
    lhTF.translation.y = environment_.lighthouses[li_it->lighthouse].translation.y;
    lhTF.translation.z = environment_.lighthouses[li_it->lighthouse].translation.z;
    lhTF.rotation.w = environment_.lighthouses[li_it->lighthouse].rotation.w;
    lhTF.rotation.x = environment_.lighthouses[li_it->lighthouse].rotation.x;
    lhTF.rotation.y = environment_.lighthouses[li_it->lighthouse].rotation.y;
    lhTF.rotation.z = environment_.lighthouses[li_it->lighthouse].rotation.z;
    // Cost related to light measurements
    if (li_it->axis == HORIZONTAL) {
      ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<ViveHorizontalCost, 4>
        (new ViveHorizontalCost(*li_it,
          lhTF,
          tracker_,
          lighthouses_[li_it->lighthouse].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(6);
      hcost->SetNumResiduals(li_it->samples.size());
      problem.AddResidualBlock(hcost, NULL, next_pose); // replace with next_pose
    } else if (li_it->axis == VERTICAL) {
      ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<ViveVerticalCost, 4>
        (new ViveVerticalCost(*li_it,
          lhTF,
          tracker_,
          lighthouses_[li_it->lighthouse].vertical_motor,
          correction_));
      vcost->AddParameterBlock(6);
      vcost->SetNumResiduals(li_it->samples.size());
      problem.AddResidualBlock(vcost, NULL, next_pose); // replace with next_pose
    }
    /* IMU */
    if (prev_pose == NULL || next_pose == NULL) continue;
    // Iterate IMU data
    while(imu_it != imu_data_.end() &&
      imu_it->header.stamp < li_it->header.stamp) {
      // Change to the next pose
      pose_it++;
      // Cost related to inertial measurements
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<InertialCost, 4, 6, 6>
        (new InertialCost(*imu_it,
          tracker_.imu_transform,
          environment_.gravity,
          DT,
          TRUST));
      problem.AddResidualBlock(cost, NULL, prev_pose, next_pose);
      imu_it++;
    }
    /* IMU */
    pose_it++;
    li_it++;
  }

  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.final_cost <<  " - "
    << poses_.back()[0] << ", "
    << poses_.back()[1] << ", "
    << poses_.back()[2] << ", "
    << poses_.back()[3] << ", "
    << poses_.back()[4] << ", "
    << poses_.back()[5] << std::endl;
  // std::cout << std::endl;
  // std::cout << summary.BriefReport() << std::endl;

  // Outlier checkers
  if (true) {
    // do something here
  }

  // // Save pose
  // pose_.header.stamp = li_it->header.stamp;
  // pose_.header.frame_id = "vive";
  // pose_.child_frame_id = tracker_.serial;
  // pose_.transform.translation.x = poses_.back()[0];
  // pose_.transform.translation.y = poses_.back()[1];
  // pose_.transform.translation.z = poses_.back()[2];
  // Eigen::Vector3d A(poses_.back()[3],
  //   poses_.back()[4],
  //   poses_.back()[5]);
  // Eigen::AngleAxisd AA(A.angle(), A.axis());
  // Eigen::Quaterniond Q(AA);
  // pose_.transform.rotation.w = Q.w();
  // pose_.transform.rotation.x = Q.x();
  // pose_.transform.rotation.y = Q.y();
  // pose_.transform.rotation.z = Q.z();

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
  if (!ViveUtils::ReadConfig(HIVE_BASE_CALIBRATION_FILE, &cal)) {
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
        4, false);
    }
  }
  ROS_INFO("Trackers' setup complete.");

  size_t counter = 0;
  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    smap[vl->header.frame_id].ProcessLight(vl);
    counter++;
    if (counter >= 100) break;
  }
  ROS_INFO("Data processment complete.");

  // Solve the refinement
  // for (auto solver : smap)
  //   solver.second.Solve();


  return 0;
}