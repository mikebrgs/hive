#include <hive/vive_pgo.h>

#define TRUST 0.4
// #define TRUST 1.6

enum DataType {imu, light};

ViveHorizontalCost::ViveHorizontalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv,
    geometry_msgs::Transform tTi,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
  tTi_ = tTi;
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
  Eigen::Matrix<T, 3, 1> vPi;
  vPi << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRi;
  ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

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

  // Inertial transform
  Eigen::Matrix<T,3,1> tPi;
  tPi << T(tTi_.translation.x),
    T(tTi_.translation.y),
    T(tTi_.translation.z);
  Eigen::Quaternion<T> tQi(
    T(tTi_.rotation.w),
    T(tTi_.rotation.x),
    T(tTi_.rotation.y),
    T(tTi_.rotation.z));
  Eigen::Matrix<T, 3, 3> tRi;
  tRi = tQi.toRotationMatrix();

  // Invert inertial transform
  Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
  Eigen::Matrix<T,3,3> iRt = tRi.transpose();

  // Vive and IMU frame to lighthouse and tracker
  Eigen::Matrix<T, 3, 1> lPv;
  lPv = - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRv;
  lRv = vRl.transpose();
  Eigen::Matrix<T, 3, 1> lPt;
  lPt = lRv * (vRi * iPt + vPi) + lPv;
  Eigen::Matrix<T, 3, 3> lRt;
  lRt = lRv * vRi * iRt;

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
  geometry_msgs::Transform tTi,
  Tracker tracker,
  Motor lighthouse,
  bool correction) {
  lighthouse_ = lighthouse;
  correction_ = correction;
  tracker_ = tracker;
  data_ = data;
  lTv_ = lTv;
  tTi_ = tTi;
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
  Eigen::Matrix<T, 3, 1> vPi;
  vPi << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> vRi;
  ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

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

  // Inertial transform
  Eigen::Matrix<T,3,1> tPi;
  tPi << T(tTi_.translation.x),
    T(tTi_.translation.y),
    T(tTi_.translation.z);
  Eigen::Quaternion<T> tQi(
    T(tTi_.rotation.w),
    T(tTi_.rotation.x),
    T(tTi_.rotation.y),
    T(tTi_.rotation.z));
  Eigen::Matrix<T, 3, 3> tRi;
  tRi = tQi.toRotationMatrix();

  // Invert inertial transform
  Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
  Eigen::Matrix<T,3,3> iRt = tRi.transpose();

  // Vive and IMU frame to lighthouse and tracker
  Eigen::Matrix<T, 3, 1> lPv;
  lPv = - vRl.transpose() * vPl;
  Eigen::Matrix<T, 3, 3> lRv;
  lRv = vRl.transpose();
  Eigen::Matrix<T, 3, 1> lPt;
  lPt = lRv * (vRi * iPt + vPi) + lPv;
  Eigen::Matrix<T, 3, 3> lRt;
  lRt = lRv * vRi * iRt;

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
  geometry_msgs::Vector3 gravity,
  geometry_msgs::Vector3 gyr_bias,
  double time_step,
  double trust_weight,
  bool verbose) {
  imu_ = imu;
  gravity_ = gravity;
  time_step_ = time_step;
  trust_weight_ = trust_weight;
  gyr_bias_ = gyr_bias;
  verbose_ = verbose;
  return;
}

InertialCost::~InertialCost() {
  // Nothing happens
  return;
}

template <typename T>
bool InertialCost::operator()(const T* const prev_vTi,
  const T* const next_vTi,
  T * residual) const {

  // prev_vTt's state
  // Position
  Eigen::Matrix<T,3,1> prev_vPi;
  prev_vPi << prev_vTi[0],
    prev_vTi[1],
    prev_vTi[2];
  // Velocity
  Eigen::Matrix<T,3,1> prev_vVi;
  prev_vVi << prev_vTi[3],
    prev_vTi[4],
    prev_vTi[5];
  // Angle axis in vector
  Eigen::Matrix<T,3,1> prev_vAi;
  prev_vAi << prev_vTi[6],
    prev_vTi[7],
    prev_vTi[8];
  // Angle axis eigen structure
  Eigen::AngleAxis<T> prev_vAAi(prev_vAi.norm(),
    prev_vAi.normalized());
  // Eigen Quaternion
  Eigen::Quaternion<T> prev_vQi(prev_vAAi);
  // Rotation matrix
  Eigen::Matrix<T,3,3> prev_vRi;// = prev_vQi.toRotationMatrix();
  ceres::AngleAxisToRotationMatrix(&prev_vTi[6], prev_vRi.data());

  // next_vTt's state
  // Position
  Eigen::Matrix<T,3,1> next_vPi;
  next_vPi << next_vTi[0],
    next_vTi[1],
    next_vTi[2];
  // Velocity
  Eigen::Matrix<T,3,1> next_vVi;
  next_vVi << next_vTi[3],
    next_vTi[4],
    next_vTi[5];
  // Angle axis in vector
  Eigen::Matrix<T,3,1> next_vAi;
  next_vAi << next_vTi[6],
    next_vTi[7],
    next_vTi[8];
  // Angle axis eigen structure
  Eigen::AngleAxis<T> next_vAAi(next_vAi.norm(),
    next_vAi.normalized());
  // Eigen Quaternion
  Eigen::Quaternion<T> next_vQi(next_vAAi);
  // Rotation matrix
  Eigen::Matrix<T,3,3> next_vRi;// = next_vQi.toRotationMatrix();
  ceres::AngleAxisToRotationMatrix(&next_vTi[6], next_vRi.data());

  // Inertial measurements
  Eigen::Matrix<T,3,1> iW;
  iW << T(imu_.angular_velocity.x),
    T(imu_.angular_velocity.y),
    T(imu_.angular_velocity.z);
  Eigen::Matrix<T,3,1> iA;
  iA << T(imu_.linear_acceleration.x),
    T(imu_.linear_acceleration.y),
    T(imu_.linear_acceleration.z);

  // Inertial bias
  Eigen::Matrix<T,3,1> iB;
  iB << T(gyr_bias_.x),
    T(gyr_bias_.y),
    T(gyr_bias_.z);

  // Gravity
  Eigen::Matrix<T,3,1> vG;
  vG << T(gravity_.x),
    T(gravity_.y),
    T(gravity_.z);

  // Inertial predictions
  // Position prediction
  Eigen::Matrix<T,3,1> est_vPi;
  est_vPi = prev_vPi + T(time_step_) * prev_vVi;// + T(0.5 * time_step_ * time_step_) * (vG - prev_vRi * iA);
  // Velocity prediction
  Eigen::Matrix<T,3,1> est_vVi;
  est_vVi = prev_vVi + T(time_step_) * (vG - (prev_vRi * iA));
  if (verbose_) {
    std::cout << "vA" << " "
      << (prev_vRi * iA)(0) << ", "
      << (prev_vRi * iA)(1) << ", "
      << (prev_vRi * iA)(2) << std::endl;
    std::cout << "vG" << " "
      << vG(0) << ", "
      << vG(1) << ", "
      << vG(2) << std::endl;
  }
  // std::cout << (prev_vRi * iA)(0) << ", " << (prev_vRi * iA)(1) << ", " << (prev_vRi * iA)(2) << std::endl;
  // Quaternion derivative matrix
  Eigen::Matrix<T,4,3> Omega;
  Omega << -prev_vQi.x(), -prev_vQi.y(), -prev_vQi.z(),
    prev_vQi.w(), -prev_vQi.z(), prev_vQi.y(),
    prev_vQi.z(), prev_vQi.w(), -prev_vQi.x(),
    -prev_vQi.y(), prev_vQi.x(), prev_vQi.w();
  // Temporary previous orientation in vector
  Eigen::Matrix<T,4,1> trev_vQi; // temporary previous
  trev_vQi << prev_vQi.w(),
    prev_vQi.x(),
    prev_vQi.y(),
    prev_vQi.z();
  Eigen::Matrix<T,4,1> text_vQi; // temporary next
  // Temporary next orientation in vector
  text_vQi = trev_vQi + T(time_step_) * T(0.5) * Omega * (iW - iB);
  // text_vQi.normalize(); // maybe remove
  // Fix small errors
  Eigen::Quaternion<T> est_vQi(text_vQi(0),
    text_vQi(1),
    text_vQi(2),
    text_vQi(3));
  Eigen::Matrix<T,3,3> est_vRi = est_vQi.toRotationMatrix();

  // Estimated from inertia vs next
  Eigen::Matrix<T,3,1> dP;
  Eigen::Matrix<T,3,1> dV;
  Eigen::Matrix<T,3,3> dR;
  dP = next_vPi - est_vPi;
  dV = next_vVi - est_vVi;
  dR = next_vRi.transpose() * est_vRi;

  // Position cost
  residual[0] = T(trust_weight_) * dP(0);
  residual[1] = T(trust_weight_) * dP(1);
  residual[2] = T(trust_weight_) * dP(2);
  // Velocity cost
  // residual[3] = prev_vPi(0) - next_vPi(0);
  // residual[4] = prev_vPi(1) - next_vPi(1);
  // residual[5] = prev_vPi(2) - next_vPi(2);
  residual[3] = T(trust_weight_) * dV(0);
  residual[4] = T(trust_weight_) * dV(1);
  residual[5] = T(trust_weight_) * dV(2);
  // Orientation cost
  T aa[3];
  ceres::RotationMatrixToAngleAxis(dR.data(), aa);
  // std::cout << aa[0] << ", " << aa[1] << ", " << aa[2] << std::endl;
  residual[6] = T(trust_weight_) *
    (aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); // WATCH OUT FOR THIS
    // Watch out for bias
  return true;
}

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
          tracker_.imu_transform,
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
          tracker_.imu_transform,
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
          tracker_.imu_transform,
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
          tracker_.imu_transform,
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