#include <hive/hive_solver.h>

// Light pose in the vive frame
class BundledHorizontalCost {
public:
  BundledHorizontalCost(hive::ViveLight data,
    Tracker tracker,
    geometry_msgs::Transform lh_pose,
    Motor lighthouse,
    bool correction);

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lh_pose_;
};

// Light pose in the vive frame
class BundledVerticalCost {
public:
  BundledVerticalCost(hive::ViveLight data,
    Tracker tracker,
    geometry_msgs::Transform lh_pose,
    Motor lighthouse,
    bool correction);

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lh_pose_;
};

BundledHorizontalCost::BundledHorizontalCost(hive::ViveLight data,
  Tracker tracker,
  geometry_msgs::Transform lh_pose,
  Motor lighthouse,
  bool correction) {
  data_ = data;
  correction_ = correction;
  tracker_ = tracker;
  lighthouse_ = lighthouse;
  lh_pose_ = lh_pose;
  }

template <typename T> bool BundledHorizontalCost::operator()(const T* const * parameters,
  T * residual) const {
  // Tracker pose
  Eigen::Matrix<T, 3, 1> vPt;
  vPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());

  // Lighthouse pose
  Eigen::Matrix<T, 3, 1> vPl;
  vPl << T(lh_pose_.translation.x),
    T(lh_pose_.translation.y),
    T(lh_pose_.translation.z);
  Eigen::Quaternion<T> vQl(T(lh_pose_.rotation.w),
    T(lh_pose_.rotation.x),
    T(lh_pose_.rotation.y),
    T(lh_pose_.rotation.z));
  Eigen::Matrix<T, 3, 3> vRl = vQl.toRotationMatrix();

  // Pose conversion
  Eigen::Matrix<T, 3, 1> lPt = vRl.transpose() * vPt - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRt = vRl.transpose() * vRt;

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

BundledVerticalCost::BundledVerticalCost(hive::ViveLight data,
  Tracker tracker,
  geometry_msgs::Transform lh_pose,
  Motor lighthouse,
  bool correction) {
  data_ = data;
  correction_ = correction;
  tracker_ = tracker;
  lighthouse_ = lighthouse;
  lh_pose_ = lh_pose;
}

template <typename T> bool BundledVerticalCost::operator()(const T* const * parameters,
  T * residual) const {
  // Tracker pose
  Eigen::Matrix<T, 3, 1> vPt;
  vPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());

  // Lighthouse pose
  Eigen::Matrix<T, 3, 1> vPl;
  vPl << T(lh_pose_.translation.x),
    T(lh_pose_.translation.y),
    T(lh_pose_.translation.z);
  Eigen::Quaternion<T> vQl(T(lh_pose_.rotation.w),
    T(lh_pose_.rotation.x),
    T(lh_pose_.rotation.y),
    T(lh_pose_.rotation.z));
  Eigen::Matrix<T, 3, 3> vRl = vQl.toRotationMatrix();

  // Pose conversion
  Eigen::Matrix<T, 3, 1> lPt = vRl.transpose() * vPt - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRt = vRl.transpose() * vRt;

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



HiveSolver::HiveSolver() {
  return;
}

HiveSolver::HiveSolver(Tracker & tracker,
    LighthouseMap & lighthouses,
    Environment & environment,
    bool correction) {
  tracker_ = tracker;
  lighthouses_ = lighthouses_;
  for (auto lighthouse : lighthouses_) {
    light_check_[lighthouse.first].first = false;
    light_check_[lighthouse.first].second = false;
  }
  environment_ = environment;
  correction_ = correction;
  valid_ = false;
  // first pose
  pose_.transform.translation.x = 0.0;
  pose_.transform.translation.y = 0.0;
  pose_.transform.translation.z = 1.0;
  pose_.transform.rotation.w = 1.0;
  pose_.transform.rotation.x = 0.0;
  pose_.transform.rotation.y = 0.0;
  pose_.transform.rotation.z = 0.0;
  return;
}

HiveSolver::~HiveSolver() {
  return;
}

void HiveSolver::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) return;

  // This solver does not use inertial measurements

  return;
}

void HiveSolver::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return;

  if (msg->axis == HORIZONTAL){
    light_data_[msg->lighthouse].first = *msg;
    light_check_[msg->lighthouse].first = true;
  } else if (msg->axis == VERTICAL) {
    light_data_[msg->lighthouse].second = *msg;
    light_check_[msg->lighthouse].second = true;
  }

  if (light_check_[msg->lighthouse].first == true
    && light_check_[msg->lighthouse].second == true) {
    valid_ = Solve();
  }

  return;
}

bool HiveSolver::GetTransform(geometry_msgs::TransformStamped &msg) {
  msg = pose_;
  return valid_;
}

bool HiveSolver::Solve() {
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Other
  ros::Time time(0);
  double n_sensors = 0;

  // Load pose
  double pose[6];
  pose[0] = pose_.transform.translation.x;
  pose[1] = pose_.transform.translation.y;
  pose[2] = pose_.transform.translation.z;
  Eigen::Quaterniond vQt_1(pose_.transform.rotation.w,
    pose_.transform.rotation.x,
    pose_.transform.rotation.y,
    pose_.transform.rotation.z);
  Eigen::AngleAxisd vAAt_1(vQt_1);
  pose[3] = vAAt_1.angle() * vAAt_1.axis()(0);
  pose[4] = vAAt_1.angle() * vAAt_1.axis()(1);
  pose[5] = vAAt_1.angle() * vAAt_1.axis()(2);

  for (auto lh_it : light_data_) {
    // Convert lighthouse transform
    geometry_msgs::Transform lighthouse;
    lighthouse.translation = environment_.lighthouses[lh_it.first].translation;
    lighthouse.rotation = environment_.lighthouses[lh_it.first].rotation;
    // Horizontal sweep
    if (light_check_[lh_it.first].first == true) {
      ceres::DynamicAutoDiffCostFunction<BundledHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<BundledHorizontalCost, 4>
        (new BundledHorizontalCost(lh_it.second.first,
          tracker_,
          lighthouse,
          lighthouses_[lh_it.first].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(6);
      hcost->SetNumResiduals(lh_it.second.first.samples.size());
      problem.AddResidualBlock(hcost, NULL, pose);
      if (lh_it.second.first.header.stamp > time)
        time = lh_it.second.first.header.stamp;
      n_sensors += lh_it.second.first.samples.size();
    }
    // Vertical sweep
    if (light_check_[lh_it.first].second == true) {
      ceres::DynamicAutoDiffCostFunction<BundledVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<BundledVerticalCost, 4>
        (new BundledVerticalCost(lh_it.second.second,
          tracker_,
          lighthouse,
          lighthouses_[lh_it.first].vertical_motor,
          correction_));
      vcost->AddParameterBlock(6);
      vcost->SetNumResiduals(lh_it.second.second.samples.size());
      problem.AddResidualBlock(vcost, NULL, pose);
      if (lh_it.second.second.header.stamp > time)
        time = lh_it.second.second.header.stamp;
      n_sensors += lh_it.second.second.samples.size();
    }
  }

  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  options.max_solver_time_in_seconds = 0.1;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.final_cost <<  " - "
    << pose[0] << ", "
    << pose[1] << ", "
    << pose[2] << ", "
    << pose[3] << ", "
    << pose[4] << ", "
    << pose[5] << std::endl;

  // Check pose
  double pose_norm = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
  if (summary.final_cost > 1e-5* n_sensors
    || pose_norm > 20
    || pose[2] <= 0 ) {
    return false;
  }


  // Save the result
  pose_.header.stamp = time;
  pose_.header.frame_id = "vive";
  pose_.child_frame_id = tracker_.serial;
  pose_.transform.translation.x = pose[0];
  pose_.transform.translation.y = pose[1];
  pose_.transform.translation.z = pose[2];
  Eigen::Vector3d vAt(pose[3], pose[4], pose[5]);
  Eigen::AngleAxisd vAAt_2;
  if (vAt.norm() != 0)
    vAAt_2 = Eigen::AngleAxisd(vAt.norm(), vAt.normalized());
  else
    vAAt_2 = Eigen::AngleAxisd(vAt.norm(), vAt);
  Eigen::Quaterniond vQt_2(vAAt_2);
  pose_.transform.rotation.w = vQt_2.w();
  pose_.transform.rotation.x = vQt_2.x();
  pose_.transform.rotation.y = vQt_2.y();
  pose_.transform.rotation.z = vQt_2.z();

  return true;
}

// int main(int argc, char ** argv) {
//   return 0;
// }