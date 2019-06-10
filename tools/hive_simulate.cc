// Includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Hive imports
#include <hive/hive_solver.h>
#include <hive/vive_filter.h>
#include <hive/vive_pgo.h>
#include <hive/vive_general.h>
#include <hive/hive_calibrator.h>
#include <hive/vive_refine.h>

// Incoming measurements
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <hive/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// C++11 includes
#include <utility>
#include <vector>
#include <tuple>
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <string>

// Poses of the tracker in the vive frame
typedef geometry_msgs::Transform (*TrajectoryFunction)(double t);
typedef std::map<std::string, Transform>::iterator TransformIterator;
// typedef std::map<size_t, std::pair<double, double>> MapPair;
typedef std::tuple<size_t, double, double> Triplet;
typedef std::vector<std::tuple<size_t, double, double>> VectorTriplet;

typedef struct State {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d angular;
} State;

class Trajectory {
private:
  ros::Time time_;
  TrajectoryFunction tfun_;
  Tracker tracker_;
  std::map<std::string, Transform> lh_poses_;
  std::map<std::string, Lighthouse> lh_specs_;
  geometry_msgs::Vector3 gravity_;
  size_t axis_;
  TransformIterator lh_pointer_;
  State this_state_;
  // Integrator
  double precision_;
  // Noise
  std::default_random_engine generator_;
  std::normal_distribution<double> acc_distribution_;
  std::normal_distribution<double> gyr_distribution_;
  std::normal_distribution<double> lig_distribution_;
  //
  bool light_used_;
public:
  Trajectory(TrajectoryFunction tfun,
    Tracker tracker,
    std::map<std::string, Transform> lh_poses,
    std::map<std::string, Lighthouse> lh_specs,
    geometry_msgs::Vector3 gravity);
  ~Trajectory();
  void Update(double dt);
  sensor_msgs::Imu::ConstPtr GetImu();
  hive::ViveLight::ConstPtr GetLight();
  geometry_msgs::TransformStamped GetTransform();
  void PrintState();
};

Trajectory::Trajectory(TrajectoryFunction tfun,
    Tracker tracker,
    std::map<std::string, Transform> lh_poses,
    std::map<std::string, Lighthouse> lh_specs,
    geometry_msgs::Vector3 gravity) {
  time_ = ros::Time(0);
  axis_ = HORIZONTAL;
  tfun_ = tfun;
  gravity_ = gravity;
  this_state_.velocity = Eigen::Vector3d::Zero();
  this_state_.acceleration = Eigen::Vector3d::Zero();
  this_state_.angular = Eigen::Vector3d::Zero();
  tracker_ = tracker;
  lh_poses_ = lh_poses;
  lh_specs_ = lh_specs;
  lh_pointer_ = lh_poses_.begin();
  precision_ = 1e-6;
  // acc_distribution_ = std::normal_distribution<double>(0,1e-10);
  // gyr_distribution_ = std::normal_distribution<double>(0,1e-10);
  // lig_distribution_ = std::normal_distribution<double>(0,1e-10);
  acc_distribution_ = std::normal_distribution<double>(0,0.02);
  gyr_distribution_ = std::normal_distribution<double>(0,7e-4);
  lig_distribution_ = std::normal_distribution<double>(0,2e-5);
  light_used_ = false;

  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();

  Eigen::Vector3d vPt(tfun(0).translation.x,
    tfun(0).translation.y,
    tfun(0).translation.z);
  Eigen::Quaterniond vQt(tfun(0).rotation.w,
    tfun(0).rotation.x,
    tfun(0).rotation.y,
    tfun(0).rotation.z);
  Eigen::Matrix3d vRt = vQt.toRotationMatrix();

  this_state_.position = vRt * tPi + vPt;
  this_state_.rotation = vRt * tRi;
  return;
}

Trajectory::~Trajectory() {
  // Do nothing
  return;
}

bool comparator(Triplet a, Triplet b) {
  return std::get<2>(a) < std::get<2>(b);
}

hive::ViveLight::ConstPtr Trajectory::GetLight() {
  hive::ViveLight * msg = new hive::ViveLight();
  // Pose of the tracker in the vive frame
  Eigen::Vector3d vPi = this_state_.position;
  Eigen::Quaterniond vQi = this_state_.rotation;
  Eigen::Matrix3d vRi = vQi.toRotationMatrix();

  // Pose of the tracker in the inertial frame
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();

  // Convert this state
  Eigen::Vector3d vPt = vRi * (-tRi.transpose() * tPi) + vPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();

  // Pose of the lighthouse in the vive frame
  Eigen::Vector3d vPl(lh_poses_[lh_pointer_->first].translation.x,
    lh_poses_[lh_pointer_->first].translation.y,
    lh_poses_[lh_pointer_->first].translation.z);
  Eigen::Quaterniond vQl(lh_poses_[lh_pointer_->first].rotation.w,
    lh_poses_[lh_pointer_->first].rotation.x,
    lh_poses_[lh_pointer_->first].rotation.y,
    lh_poses_[lh_pointer_->first].rotation.z);
  Eigen::Matrix3d vRl = vQl.toRotationMatrix();

  // Convert poses
  Eigen::Vector3d lPt = vRl.transpose() * vPt - vRl.transpose() * vPl;
  Eigen::Matrix3d lRt = vRl.transpose() * vRt;

  // Extrinsics
  double phase;
  double tilt;
  double gib_phase;
  double gib_mag;
  double curve;
  if (axis_ == HORIZONTAL) {
    phase = lh_specs_[lh_pointer_->first].horizontal_motor.phase;
    tilt = lh_specs_[lh_pointer_->first].horizontal_motor.tilt;
    gib_phase = lh_specs_[lh_pointer_->first].horizontal_motor.gib_phase;
    gib_mag = lh_specs_[lh_pointer_->first].horizontal_motor.gib_magnitude;
    curve = lh_specs_[lh_pointer_->first].horizontal_motor.curve;
  } else {
    phase = lh_specs_[lh_pointer_->first].vertical_motor.phase;
    tilt = lh_specs_[lh_pointer_->first].vertical_motor.tilt;
    gib_phase = lh_specs_[lh_pointer_->first].vertical_motor.gib_phase;
    gib_mag = lh_specs_[lh_pointer_->first].vertical_motor.gib_magnitude;
    curve = lh_specs_[lh_pointer_->first].vertical_motor.curve;
  }

  VectorTriplet data;
  for (auto sensor : tracker_.sensors) {
    Eigen::Vector3d tPs(sensor.second.position.x,
      sensor.second.position.y,
      sensor.second.position.z);
    Eigen::Vector3d tNs(sensor.second.normal.x,
      sensor.second.normal.y,
      sensor.second.normal.z);
    Eigen::Vector3d lPs = lRt * tPs + lPt;

    double dproduct = lPs.normalized().transpose() * tNs.normalized();

    double angle;
    double x = (lPs(0)/lPs(2)); // Horizontal angle
    double y = (lPs(1)/lPs(2)); // Vertical angle

    if (axis_ == HORIZONTAL) {
      angle = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
    } else {
      angle = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
    }

    if (angle > M_PI/3 || angle < -M_PI/3)
      continue;

    data.push_back(std::make_tuple(
      sensor.first,
      angle + lig_distribution_(generator_),
      dproduct));
  }

  std::sort(data.begin(), data.end(), comparator);

  for (size_t i = 0; i < 24; i++) {
    if (std::get<2>(data[i]) > 0) break;
    hive::ViveLightSample * sample_msg = new hive::ViveLightSample();
    sample_msg->sensor = std::get<0>(data[i]);
    sample_msg->angle = std::get<1>(data[i]);
    msg->samples.push_back(*sample_msg);
  }

  msg->lighthouse = lh_pointer_->first;
  msg->header.frame_id = tracker_.serial;
  msg->header.stamp = time_;
  msg->axis = static_cast<uint8_t>(axis_);

  light_used_ = true;

  return hive::ViveLight::ConstPtr(msg);
}

sensor_msgs::Imu::ConstPtr Trajectory::GetImu() {
  sensor_msgs::Imu * msg = new sensor_msgs::Imu();

  msg->linear_acceleration.x = this_state_.acceleration(0)
    + acc_distribution_(generator_);
  msg->linear_acceleration.y = this_state_.acceleration(1)
    + acc_distribution_(generator_);
  msg->linear_acceleration.z = this_state_.acceleration(2)
    + acc_distribution_(generator_);

  msg->angular_velocity.x = this_state_.angular(0)
    + gyr_distribution_(generator_);
  msg->angular_velocity.y = this_state_.angular(1)
    + gyr_distribution_(generator_);
  msg->angular_velocity.z = this_state_.angular(2)
    + gyr_distribution_(generator_);

  msg->header.frame_id = tracker_.serial;
  msg->header.stamp = time_;

  return sensor_msgs::Imu::ConstPtr(msg);
}

void Trajectory::Update(double dt) {
  time_ = time_ + ros::Duration(dt);

  // Tracker params
  Eigen::Vector3d Ba(tracker_.acc_bias.x,
    tracker_.acc_bias.y,
    tracker_.acc_bias.z);
  Eigen::Vector3d Bw(tracker_.gyr_bias.x,
    tracker_.gyr_bias.y,
    tracker_.gyr_bias.z);
  Eigen::Vector3d vG(gravity_.x,
    gravity_.y,
    gravity_.z);

  // Pose of the tracker in the inertial frame
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();

  // Poses
  geometry_msgs::Transform msg3 = tfun_(time_.toSec() - 2.0 * precision_);
  geometry_msgs::Transform msg2 = tfun_(time_.toSec() - 1.0 * precision_);
  geometry_msgs::Transform msg1 = tfun_(time_.toSec() - 0.0 * precision_);

  // Orientations
  Eigen::Quaterniond vQt_1(msg1.rotation.w,
    msg1.rotation.x,
    msg1.rotation.y,
    msg1.rotation.z);
  Eigen::Matrix3d vRt_1 = vQt_1.toRotationMatrix();
  Eigen::Matrix3d vRi_1 = vRt_1 * tRi;
  Eigen::Quaterniond vQi_1(vRi_1);
  Eigen::Vector4d vQVi_1(vQi_1.w(),
    vQi_1.x(),
    vQi_1.y(),
    vQi_1.z());
  Eigen::Quaterniond vQt_2(msg2.rotation.w,
    msg2.rotation.x,
    msg2.rotation.y,
    msg2.rotation.z);
  Eigen::Matrix3d vRt_2 = vQt_2.toRotationMatrix();
  Eigen::Matrix3d vRi_2 = vRt_2 * tRi;
  Eigen::Quaterniond vQi_2(vRi_2);
  Eigen::Vector4d vQVi_2(vQi_2.w(),
    vQi_2.x(),
    vQi_2.y(),
    vQi_2.z());
  Eigen::Quaterniond vQt_3(msg3.rotation.w,
    msg3.rotation.x,
    msg3.rotation.y,
    msg3.rotation.z);
  Eigen::Matrix3d vRt_3 = vQt_3.toRotationMatrix();

  // Positions
  Eigen::Vector3d vPt_1(msg1.translation.x,
    msg1.translation.y,
    msg1.translation.z);
  Eigen::Vector3d vPi_1 = vRt_1 * tPi + vPt_1;
  Eigen::Vector3d vPt_2(msg2.translation.x,
    msg2.translation.y,
    msg2.translation.z);
  Eigen::Vector3d vPi_2 = vRt_2 * tPi + vPt_2;
  Eigen::Vector3d vPt_3(msg3.translation.x,
    msg3.translation.y,
    msg3.translation.z);
  Eigen::Vector3d vPi_3 = vRt_3 * tPi + vPt_3;

  // Linear velocities
  Eigen::Vector3d vVi_1 = (vPi_1 - vPi_2) / precision_;
  Eigen::Vector3d vVi_2 = (vPi_2 - vPi_3) / precision_;

  // Diff
  Eigen::Vector4d vDQi = (vQVi_1 - vQVi_2) / precision_;
  Eigen::Matrix3d vRi = Eigen::Quaterniond(vQi_1.w(),
    vQi_1.x(),
    vQi_1.y(),
    vQi_1.z()).toRotationMatrix();

  //0.5 * Omega matrix
  Eigen::Matrix<double, 4, 3> A;
  A(0,0) = -vQi_1.x();
  A(0,1) = -vQi_1.y();
  A(0,2) = -vQi_1.z();
  A(1,0) = vQi_1.w();
  A(1,1) = -vQi_1.z();
  A(1,2) = vQi_1.y();
  A(2,0) = vQi_1.z();
  A(2,1) = vQi_1.w();
  A(2,2) = -vQi_1.x();
  A(3,0) = -vQi_1.y();
  A(3,1) = vQi_1.x();
  A(3,2) = vQi_1.w();
  A = 0.5 * A;

  // Angular velocity
  Eigen::Vector3d Wi = (A.transpose() * A).inverse() * A.transpose() * vDQi + Bw;

  // Accelerations
  Eigen::Vector3d Ai = vRi.transpose() * (vG - (vVi_1 - vVi_2) / precision_) + Ba;


  // Update lighthouses
  if (light_used_ = true) {
    if (axis_ >= 1) {
      axis_ = 0;
      lh_pointer_++;
      if (lh_pointer_ == lh_poses_.end()) {
        lh_pointer_ = lh_poses_.begin();
      }
    } else {
      axis_ = 1;
    }
  }

  // Final
  this_state_.position = vPi_1;
  this_state_.velocity = vVi_1;
  this_state_.acceleration = Ai;
  this_state_.rotation = Eigen::Quaterniond(
    vQi_1.w(),
    vQi_1.x(),
    vQi_1.y(),
    vQi_1.z());
  this_state_.angular = Wi;
  light_used_ = false;

  return;
}

geometry_msgs::TransformStamped Trajectory::GetTransform() {
  geometry_msgs::TransformStamped msg;


  // Pose of the tracker in the vive frame
  Eigen::Vector3d vPi = this_state_.position;
  Eigen::Quaterniond vQi = this_state_.rotation;
  Eigen::Matrix3d vRi = vQi.toRotationMatrix();

  // Pose of the tracker in the inertial frame
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();

  // Convert this state
  Eigen::Vector3d vPt = vRi * (-tRi.transpose() * tPi) + vPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();
  Eigen::Quaterniond vQt(vRt);

  msg.transform.translation.x = vPt(0);
  msg.transform.translation.y = vPt(1);
  msg.transform.translation.z = vPt(2);

  msg.transform.rotation.w = vQt.w();
  msg.transform.rotation.x = vQt.x();
  msg.transform.rotation.y = vQt.y();
  msg.transform.rotation.z = vQt.z();

  msg.child_frame_id = tracker_.serial;
  msg.header.stamp = time_;
  msg.header.frame_id = "vive";

  return msg;
}

void Trajectory::PrintState() {
  std::cout << "P: " << this_state_.position(0) << ", "
    << this_state_.position(1) << ", "
    << this_state_.position(2) << std::endl;
  std::cout << "V: " << this_state_. velocity(0) << ", "
    << this_state_.velocity(1) << ", "
    << this_state_.velocity(2) << std::endl;
  std::cout << "A: " << this_state_.acceleration(0) << ", "
    << this_state_.acceleration(1) << ", "
    << this_state_.acceleration(2) << std::endl;
  std::cout << "Q: " << this_state_.rotation.w() << ", "
    << this_state_.rotation.x() << ", "
    << this_state_.rotation.y() << ", "
    << this_state_.rotation.z() << std::endl;
  std::cout << "W: " << this_state_.angular(0) << ", "
    << this_state_.angular(1) << ", "
    << this_state_.angular(2) << std::endl;
  return;
}

geometry_msgs::Transform trajectory1(double t) {
  geometry_msgs::Transform msg;
  msg.translation.x = 0.0;
  msg.translation.y = 0.0;
  msg.translation.z = 2.0;

  msg.rotation.w = 1.0;
  msg.rotation.x = 0.0;
  msg.rotation.y = 0.0;
  msg.rotation.z = 0.0;

  return msg;
}

geometry_msgs::Transform trajectory2(double t) {
  geometry_msgs::Transform msg;
  msg.translation.x = 0.0;
  msg.translation.y = 0.0;
  msg.translation.z = 1.0 + 0.1 * t;

  msg.rotation.w = 1.0;
  msg.rotation.x = 0.0;
  msg.rotation.y = 0.0;
  msg.rotation.z = 0.0;

  return msg;
}

geometry_msgs::Transform trajectory3(double t) {
  geometry_msgs::Transform msg;
  double v, w;
  v = 0.05;
  w = 0.05;
  msg.translation.x = cos(2*M_PI * v * t);
  msg.translation.y = sin(2*M_PI * v * t);
  msg.translation.z = 2.0;

  Eigen::Vector3d vAi(0,0,1);
  Eigen::AngleAxisd vAAi(2*M_PI * w * t, vAi);
  Eigen::Quaterniond vQi(vAAi);

  msg.rotation.w = vQi.w();
  msg.rotation.x = vQi.x();
  msg.rotation.y = vQi.y();
  msg.rotation.z = vQi.z();

  return msg;
}


geometry_msgs::Transform trajectory4(double t) {
  geometry_msgs::Transform msg;
  double v, w;
  v = 0.1;
  w = 0.1;
  msg.translation.x = cos(2*M_PI * v * t);
  msg.translation.y = sin(2*M_PI * v * t);
  msg.translation.z = 2.0;

  Eigen::Vector3d vAi(0,0,1);
  Eigen::AngleAxisd vAAi(2*M_PI * w * t, vAi);
  Eigen::Quaterniond vQi(vAAi);

  msg.rotation.w = vQi.w();
  msg.rotation.x = vQi.x();
  msg.rotation.y = vQi.y();
  msg.rotation.z = vQi.z();

  return msg;
}


// Main function
int main(int argc, char ** argv) {
  // Data
  Calibration calibration, new_calibration;
  // std::map<std::string, Solver*> solver;
  // std::map<std::string, Solver*> aux_solver;

  // Read bag with data
  if (argc < 3) {
    std::cout << "Usage: ... hive_calibrator read.bag write.bag" << std::endl;
    return -1;
  }
  rosbag::Bag rbag, wbag;
  rosbag::View view;
  std::string read_bag(argv[1]);
  std::string write_bag(argv[2]);
  rbag.open(read_bag, rosbag::bagmode::Read);
  wbag.open(write_bag, rosbag::bagmode::Write);

  ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE,
    &calibration);

  // Get current calibration
  ROS_INFO("Reading JSON file.");
  JsonParser jp = JsonParser(HIVE_CONFIG_FILE);
  jp.GetBody(&calibration);

  ViveCalibrate calibrator(calibration, true);

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    calibration.SetLighthouses(*vl);
    calibrator.Update(vl);
  }
  // geometry_msgs::Vector3 gravity;
  // gravity.x = 0.0;
  // gravity.y = 9.8;
  // gravity.z = 0.0;
  // calibration.environment.gravity = gravity;
  // Transform lh_pose_1;
  // lh_pose_1.parent_frame = "vive";
  // lh_pose_1.child_frame = "lh1";
  // lh_pose_1.translation.x = 0.0;
  // lh_pose_1.translation.y = 0.0;
  // lh_pose_1.translation.z = 0.0;
  // lh_pose_1.rotation.w = 1.0;
  // lh_pose_1.rotation.x = 0.0;
  // lh_pose_1.rotation.y = 0.0;
  // lh_pose_1.rotation.z = 0.0;
  // calibration.environment.lighthouses[lh_pose_1.child_frame] = lh_pose_1;
  // Transform lh_pose_2;
  // lh_pose_2.parent_frame = "vive";
  // lh_pose_2.child_frame = "lh2";
  // lh_pose_2.translation.x = 1.414213562373095;
  // lh_pose_2.translation.y = 1.414213562373095;
  // lh_pose_2.translation.z = 2.0;
  // lh_pose_2.rotation.w = 0.653281482438188;
  // lh_pose_2.rotation.x = 0.270598050073098;
  // lh_pose_2.rotation.y = -0.653281482438188;
  // lh_pose_2.rotation.z = 0.270598050073098;
  // calibration.environment.lighthouses[lh_pose_2.child_frame] = lh_pose_2;
  // Transform body;
  // body.parent_frame = "LHR-FD35B946";
  // body.child_frame = "marker";
  // body.translation.x = 0.0;
  // body.translation.y = 0.0;
  // body.translation.z = 0.0;
  // body.rotation.w = 1.0;
  // body.rotation.x = 0.0;
  // body.rotation.y = 0.0;
  // body.rotation.z = 0.0;
  // calibration.environment.bodies[lh_pose_1.child_frame] = body;
  // Transform offset;
  // offset.parent_frame = "";
  // offset.child_frame = "";
  // offset.translation.x = 0.0;
  // offset.translation.y = 0.0;
  // offset.translation.z = 0.0;
  // offset.rotation.w = 1.0;
  // offset.rotation.x = 0.0;
  // offset.rotation.y = 0.0;
  // offset.rotation.z = 0.0;
  // calibration.environment.offset = offset;
  // Transform vive;
  // vive.parent_frame = "world";
  // vive.child_frame = "vive";
  // vive.translation.x = 0.0;
  // vive.translation.y = 0.0;
  // vive.translation.z = 0.0;
  // vive.rotation.w = 1.0;
  // vive.rotation.x = 0.0;
  // vive.rotation.y = 0.0;
  // vive.rotation.z = 0.0;
  // calibration.environment.vive = vive;
  // Lighthouse lh_specs_1;
  // lh_specs_1.serial = lh_pose_1.child_frame;
  // lh_specs_1.horizontal_motor.phase = 0.0;
  // lh_specs_1.horizontal_motor.tilt = 0.0;
  // lh_specs_1.horizontal_motor.gib_phase = 0.0;
  // lh_specs_1.horizontal_motor.gib_magnitude = 0.0;
  // lh_specs_1.horizontal_motor.curve = 0.0;
  // lh_specs_1.vertical_motor.phase = 0.0;
  // lh_specs_1.vertical_motor.tilt = 0.0;
  // lh_specs_1.vertical_motor.gib_phase = 0.0;
  // lh_specs_1.vertical_motor.gib_magnitude = 0.0;
  // lh_specs_1.vertical_motor.curve = 0.0;
  // calibration.lighthouses[lh_specs_1.serial] = lh_specs_1;
  // Lighthouse lh_specs_2;
  // lh_specs_2.serial = lh_pose_1.child_frame;
  // lh_specs_2.horizontal_motor.phase = 0.0;
  // lh_specs_2.horizontal_motor.tilt = 0.0;
  // lh_specs_2.horizontal_motor.gib_phase = 0.0;
  // lh_specs_2.horizontal_motor.gib_magnitude = 0.0;
  // lh_specs_2.horizontal_motor.curve = 0.0;
  // lh_specs_2.vertical_motor.phase = 0.0;
  // lh_specs_2.vertical_motor.tilt = 0.0;
  // lh_specs_2.vertical_motor.gib_phase = 0.0;
  // lh_specs_2.vertical_motor.gib_magnitude = 0.0;
  // lh_specs_2.vertical_motor.curve = 0.0;
  // calibration.lighthouses[lh_specs_2.serial] = lh_specs_2;
  // calibration.environment.gravity
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    calibration.SetTrackers(*vt);
    calibrator.Update(vt);
  }
  ROS_INFO("Trackers' setup complete.");

  double Tl = 1.0e0/120.0;

  Tracker tracker = calibration.trackers.begin()->second;


  // Calibration

  Trajectory tr_cal(&trajectory1,
    tracker,
    calibration.environment.lighthouses,
    calibration.lighthouses,
    calibration.environment.gravity);
  for (size_t i = 0; i <= 200; i++) {
    hive::ViveLight::ConstPtr vl = tr_cal.GetLight();

    calibrator.AddLight(vl);

    sensor_msgs::Imu::ConstPtr vi;
    tr_cal.Update(Tl / 4.0);
    vi = tr_cal.GetImu();
    calibrator.AddImu(vi);
    tr_cal.Update(Tl / 2.0);
    vi = tr_cal.GetImu();
    calibrator.AddImu(vi);
    tr_cal.Update(Tl / 4.0);

  }

  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> gt_lighthouses;
  for (auto lighthouse : calibration.environment.lighthouses) {
    gt_lighthouses[lighthouse.first].first = Eigen::Vector3d(
      lighthouse.second.translation.x,
      lighthouse.second.translation.y,
      lighthouse.second.translation.z);
    gt_lighthouses[lighthouse.first].second = Eigen::Quaterniond(
      lighthouse.second.rotation.w,
      lighthouse.second.rotation.x,
      lighthouse.second.rotation.y,
      lighthouse.second.rotation.z);
  }
  Eigen::Vector3d gt_g(calibration.environment.gravity.x,
    calibration.environment.gravity.y,
    calibration.environment.gravity.z);

  calibrator.Solve();

  new_calibration = calibrator.GetCalibration();

  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> est_lighthouses_cal;
  for (auto lighthouse : new_calibration.environment.lighthouses) {
    est_lighthouses_cal[lighthouse.first].first = Eigen::Vector3d(
      lighthouse.second.translation.x,
      lighthouse.second.translation.y,
      lighthouse.second.translation.z);
    est_lighthouses_cal[lighthouse.first].second = Eigen::Quaterniond(
      lighthouse.second.rotation.w,
      lighthouse.second.rotation.x,
      lighthouse.second.rotation.y,
      lighthouse.second.rotation.z);
    std::cout << "dP: "
      << (est_lighthouses_cal[lighthouse.first].first -
      gt_lighthouses[lighthouse.first].first).norm() << std::endl;
    std::cout << "dA: "
      << 180.0 / M_PI * Eigen::AngleAxisd(
        est_lighthouses_cal[lighthouse.first].second.toRotationMatrix().transpose() *
        gt_lighthouses[lighthouse.first].second.toRotationMatrix()).angle() << std::endl;
  }
  Eigen::Vector3d est_g_cal(new_calibration.environment.gravity.x,
    new_calibration.environment.gravity.y,
    new_calibration.environment.gravity.z);
    std::cout << "dG: " << (est_g_cal - gt_g).norm() << std::endl;

  Refinery refinery(new_calibration, true, 1e1, true);
  Trajectory tr_ref(&trajectory4,
    tracker,
    calibration.environment.lighthouses,
    calibration.lighthouses,
    calibration.environment.gravity);

  for (size_t i = 0; i <= 1200; i++) {
    hive::ViveLight::ConstPtr vl = tr_ref.GetLight();

    refinery.AddLight(vl);

    sensor_msgs::Imu::ConstPtr vi;
    tr_ref.Update(Tl / 4.0);
    vi = tr_ref.GetImu();
    refinery.AddImu(vi);
    tr_ref.Update(Tl / 2.0);
    vi = tr_ref.GetImu();
    refinery.AddImu(vi);
    tr_ref.Update(Tl / 4.0);

  }

  refinery.Solve();

  new_calibration = refinery.GetCalibration();

  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> est_lighthouses_ref;
  for (auto lighthouse : new_calibration.environment.lighthouses) {
    est_lighthouses_ref[lighthouse.first].first = Eigen::Vector3d(
      lighthouse.second.translation.x,
      lighthouse.second.translation.y,
      lighthouse.second.translation.z);
    est_lighthouses_ref[lighthouse.first].second = Eigen::Quaterniond(
      lighthouse.second.rotation.w,
      lighthouse.second.rotation.x,
      lighthouse.second.rotation.y,
      lighthouse.second.rotation.z);
    std::cout << "dP: "
      << (est_lighthouses_ref[lighthouse.first].first -
      gt_lighthouses[lighthouse.first].first).norm() << std::endl;
    std::cout << "dA: "
      << 180.0 / M_PI * Eigen::AngleAxisd(
        est_lighthouses_ref[lighthouse.first].second.toRotationMatrix().transpose() *
        gt_lighthouses[lighthouse.first].second.toRotationMatrix()).angle() << std::endl;
  }
  Eigen::Vector3d est_g_ref(new_calibration.environment.gravity.x,
    new_calibration.environment.gravity.y,
    new_calibration.environment.gravity.z);
    std::cout << "dG: " << (est_g_ref - gt_g).norm() << std::endl;



  return 0;

  // Tracking
  // double Tl = 1.0e0/120.0;

  Trajectory tr(&trajectory1,
    tracker,
    calibration.environment.lighthouses,
    calibration.lighthouses,
    calibration.environment.gravity);

  std::vector<Eigen::Vector3d> gt_positions;
  std::vector<Eigen::Vector3d> gt_attitudes;

  Solver * solver_ape1 = new HiveSolver(tracker,
    calibration.lighthouses,
    calibration.environment,
    false);
  std::vector<Eigen::Vector3d> ape1_positions;
  std::vector<Eigen::Vector3d> ape1_attitudes;
  std::vector<double> ape1_distances;
  std::vector<double> ape1_angles;
  Solver * solver_ape2 = new HiveSolver(tracker,
    calibration.lighthouses,
    calibration.environment,
    true);
  std::vector<Eigen::Vector3d> ape2_positions;
  std::vector<Eigen::Vector3d> ape2_attitudes;
  std::vector<double> ape2_distances;
  std::vector<double> ape2_angles;
  Solver * solver_ekf = new ViveFilter(tracker,
    calibration.lighthouses,
    calibration.environment,
    1e-1, 1e-6, true, filter::ekf);
  std::vector<Eigen::Vector3d> ekf_positions;
  std::vector<Eigen::Vector3d> ekf_attitudes;
  std::vector<double> ekf_distances;
  std::vector<double> ekf_angles;
  Solver * solver_iekf = new ViveFilter(tracker,
    calibration.lighthouses,
    calibration.environment,
    1e-1, 1e-6, true, filter::iekf);
  std::vector<Eigen::Vector3d> iekf_positions;
  std::vector<Eigen::Vector3d> iekf_attitudes;
  std::vector<double> iekf_distances;
  std::vector<double> iekf_angles;
  Solver * solver_ukf = new ViveFilter(tracker,
    calibration.lighthouses,
    calibration.environment,
    1.0e0, 1e-6, true, filter::ukf);
  std::vector<Eigen::Vector3d> ukf_positions;
  std::vector<Eigen::Vector3d> ukf_attitudes;
  std::vector<double> ukf_distances;
  std::vector<double> ukf_angles;
  Solver * solver_pgo = new PoseGraph(calibration.environment,
    tracker,
    calibration.lighthouses,
    4, 7e-4, 1e0, true);
  std::vector<Eigen::Vector3d> pgo_positions;
  std::vector<Eigen::Vector3d> pgo_attitudes;
  std::vector<double> pgo_distances;
  std::vector<double> pgo_angles;


  // 2400 for trajectory3
  geometry_msgs::TransformStamped msg, gt_msg;
  for (size_t i = 0; i <= 2400; i++) {
    hive::ViveLight::ConstPtr vl = tr.GetLight();
    solver_ape1->ProcessLight(vl);
    solver_ape2->ProcessLight(vl);
    solver_ekf->ProcessLight(vl);
    solver_iekf->ProcessLight(vl);
    solver_ukf->ProcessLight(vl);
    solver_pgo->ProcessLight(vl);
    gt_msg = tr.GetTransform();
    Eigen::Vector3d gt_P(gt_msg.transform.translation.x,
      gt_msg.transform.translation.y,
      gt_msg.transform.translation.z);
    Eigen::Matrix3d gt_R = Eigen::Quaterniond(
      gt_msg.transform.rotation.w,
      gt_msg.transform.rotation.x,
      gt_msg.transform.rotation.y,
      gt_msg.transform.rotation.z).toRotationMatrix();
    Eigen::AngleAxisd gt_AA(gt_R);
    std::cout << "GT" << " - "
      << gt_msg.child_frame_id << " : "
      << gt_msg.transform.translation.x << ", "
      << gt_msg.transform.translation.y << ", "
      << gt_msg.transform.translation.z << ", "
      << gt_msg.transform.rotation.w << ", "
      << gt_msg.transform.rotation.x << ", "
      << gt_msg.transform.rotation.y << ", "
      << gt_msg.transform.rotation.z << std::endl;
    gt_positions.push_back(gt_P);
    gt_attitudes.push_back(gt_AA.axis() * gt_AA.angle());

    if (solver_ape1->GetTransform(msg)) {
      ape1_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd ape1_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      ape1_attitudes.push_back(ape1_AA.axis() * ape1_AA.angle());
      ape1_distances.push_back((ape1_positions.back() - gt_P).norm());
      Eigen::Matrix3d ape1_R = ape1_AA.toRotationMatrix();
      ape1_angles.push_back(
        Eigen::AngleAxisd(ape1_R.transpose() * gt_R).angle());
      std::cout << "APE1" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;
    }
    if (solver_ape2->GetTransform(msg)) {
      ape2_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd ape2_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      ape2_attitudes.push_back(ape2_AA.axis() * ape2_AA.angle());
      ape2_distances.push_back((ape2_positions.back() - gt_P).norm());
      Eigen::Matrix3d ape2_R = ape2_AA.toRotationMatrix();
      ape2_angles.push_back(
        Eigen::AngleAxisd(ape2_R.transpose() * gt_R).angle());
      std::cout << "APE2" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;

    }
    if (solver_ekf->GetTransform(msg)) {
      ekf_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd ekf_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      ekf_attitudes.push_back(ekf_AA.axis() * ekf_AA.angle());
      ekf_distances.push_back((ekf_positions.back() - gt_P).norm());
      Eigen::Matrix3d ekf_R = ekf_AA.toRotationMatrix();
      ekf_angles.push_back(
        Eigen::AngleAxisd(ekf_R.transpose() * gt_R).angle());
      std::cout << "EKF" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;

    }
    if (solver_iekf->GetTransform(msg)) {
      iekf_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd iekf_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      iekf_attitudes.push_back(iekf_AA.axis() * iekf_AA.angle());
      iekf_distances.push_back((iekf_positions.back() - gt_P).norm());
      Eigen::Matrix3d iekf_R = iekf_AA.toRotationMatrix();
      iekf_angles.push_back(
        Eigen::AngleAxisd(iekf_R.transpose() * gt_R).angle());
      std::cout << "IEKF" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;

    }
    if (solver_ukf->GetTransform(msg)) {
      ukf_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd ukf_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      ukf_attitudes.push_back(ukf_AA.axis() * ukf_AA.angle());
      ukf_distances.push_back((ukf_positions.back() - gt_P).norm());
      Eigen::Matrix3d ukf_R = ukf_AA.toRotationMatrix();
      ukf_angles.push_back(
        Eigen::AngleAxisd(ukf_R.transpose() * gt_R).angle());
      std::cout << "UKF" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;

    }
    if (solver_pgo->GetTransform(msg)) {
      pgo_positions.push_back(Eigen::Vector3d(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z));
      Eigen::AngleAxisd pgo_AA(Eigen::Quaterniond(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z));
      pgo_attitudes.push_back(pgo_AA.axis() * pgo_AA.angle());
      pgo_distances.push_back((pgo_positions.back() - gt_P).norm());
      Eigen::Matrix3d pgo_R = pgo_AA.toRotationMatrix();
      pgo_angles.push_back(
        Eigen::AngleAxisd(pgo_R.transpose() * gt_R).angle());
      std::cout << "PGO" << " - "
        << msg.child_frame_id << " : "
        << msg.transform.translation.x << ", "
        << msg.transform.translation.y << ", "
        << msg.transform.translation.z << ", "
        << msg.transform.rotation.w << ", "
        << msg.transform.rotation.x << ", "
        << msg.transform.rotation.y << ", "
        << msg.transform.rotation.z << std::endl;

    }

    sensor_msgs::Imu::ConstPtr vi;
    tr.Update(Tl / 4.0);
    vi = tr.GetImu();
    std::cout << "IMU "
      << vi->linear_acceleration.x << ", "
      << vi->linear_acceleration.y << ", "
      << vi->linear_acceleration.z << ", "
      << vi->angular_velocity.x << ", "
      << vi->angular_velocity.y << ", "
      << vi->angular_velocity.z << std::endl;
    solver_ape1->ProcessImu(vi);
    solver_ape2->ProcessImu(vi);
    solver_ekf->ProcessImu(vi);
    solver_iekf->ProcessImu(vi);
    solver_ukf->ProcessImu(vi);
    solver_pgo->ProcessImu(vi);
    tr.Update(Tl / 2.0);
    vi = tr.GetImu();
    std::cout << "IMU "
      << vi->linear_acceleration.x << ", "
      << vi->linear_acceleration.y << ", "
      << vi->linear_acceleration.z << ", "
      << vi->angular_velocity.x << ", "
      << vi->angular_velocity.y << ", "
      << vi->angular_velocity.z << std::endl;
    solver_ape1->ProcessImu(vi);
    solver_ape2->ProcessImu(vi);
    solver_ekf->ProcessImu(vi);
    solver_iekf->ProcessImu(vi);
    solver_ukf->ProcessImu(vi);
    solver_pgo->ProcessImu(vi);
    tr.Update(Tl / 4.0);

    std::cout << std::endl;
  }


  {
    std::cout << "GT" << std::endl;
    for (auto position : gt_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : gt_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : gt_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : gt_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : gt_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : gt_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "APE1" << std::endl;
    for (auto position : ape1_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ape1_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ape1_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape1_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape1_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape1_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "APE2" << std::endl;
    for (auto position : ape2_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ape2_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ape2_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape2_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape2_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ape2_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "EKF" << std::endl;
    for (auto position : ekf_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ekf_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ekf_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ekf_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ekf_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ekf_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "IEKF" << std::endl;
    for (auto position : iekf_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : iekf_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : iekf_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : iekf_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : iekf_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : iekf_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "UKF" << std::endl;
    for (auto position : ukf_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ukf_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : ukf_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ukf_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ukf_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : ukf_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  {
    std::cout << "PGOW" << std::endl;
    for (auto position : pgo_positions) {
      std::cout << position[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : pgo_positions) {
      std::cout << position[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto position : pgo_positions) {
      std::cout << position[2] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : pgo_attitudes) {
      std::cout << attitude[0] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : pgo_attitudes) {
      std::cout << attitude[1] << " ";
    }
    std::cout << std::endl << std::endl;
    for (auto attitude : pgo_attitudes) {
      std::cout << attitude[2] << " ";
    }
    std::cout << std::endl << std::endl;
  }

  // Moving results
  {
    double ape1_mean_distance = 0;
    double ape1_max_distance = 0;
    for (auto distance : ape1_distances) {
      ape1_mean_distance += distance;
      if (distance > ape1_max_distance)
        ape1_max_distance = distance;
    }
    ape1_mean_distance = ape1_mean_distance / (double) ape1_distances.size();
    std::cout << "APE1 mean distance " << ape1_mean_distance << std::endl;
    std::cout << "APE1 max distance " << ape1_max_distance << std::endl;

    double ape1_mean_angle = 0;
    double ape1_max_angle = 0;
    for (auto angle : ape1_angles) {
      ape1_mean_angle += angle;
      if (angle > ape1_max_angle)
        ape1_max_angle = angle;
    }
    ape1_mean_angle = ape1_mean_angle / (double) ape1_angles.size();
    std::cout << "APE1 mean angle " << 180.0 / M_PI * ape1_mean_angle << std::endl;
    std::cout << "APE1 max angle " << 180.0 / M_PI * ape1_max_angle << std::endl;
  }

  {
    double ape2_mean_distance = 0;
    double ape2_max_distance = 0;
    for (auto distance : ape2_distances) {
      ape2_mean_distance += distance;
      if (distance > ape2_max_distance)
        ape2_max_distance = distance;
    }
    ape2_mean_distance = ape2_mean_distance / (double) ape2_distances.size();
    std::cout << "APE2 mean distance " << ape2_mean_distance << std::endl;
    std::cout << "APE2 max distance " << ape2_max_distance << std::endl;

    double ape2_mean_angle = 0;
    double ape2_max_angle = 0;
    for (auto angle : ape2_angles) {
      ape2_mean_angle += angle;
      if (angle > ape2_max_angle)
        ape2_max_angle = angle;
    }
    ape2_mean_angle = ape2_mean_angle / (double) ape2_angles.size();
    std::cout << "APE2 mean angle " << 180.0 / M_PI * ape2_mean_angle << std::endl;
    std::cout << "APE2 max angle " << 180.0 / M_PI * ape2_max_angle << std::endl;
  }

  {
    double ekf_mean_distance = 0;
    double ekf_max_distance = 0;
    for (auto distance : ekf_distances) {
      ekf_mean_distance += distance;
      if (distance > ekf_max_distance)
        ekf_max_distance = distance;
    }
    ekf_mean_distance = ekf_mean_distance / (double) ekf_distances.size();
    std::cout << "EKF mean distance " << ekf_mean_distance << std::endl;
    std::cout << "EKF max distance " << ekf_max_distance << std::endl;

    double ekf_mean_angle = 0;
    double ekf_max_angle = 0;
    for (auto angle : ekf_angles) {
      ekf_mean_angle += angle;
      if (angle > ekf_max_angle)
        ekf_max_angle = angle;
    }
    ekf_mean_angle = ekf_mean_angle / (double) ekf_angles.size();
    std::cout << "EKF mean angle " << 180.0 / M_PI * ekf_mean_angle << std::endl;
    std::cout << "EKF max angle " << 180.0 / M_PI * ekf_max_angle << std::endl;
  }

  {
    double iekf_mean_distance = 0;
    double iekf_max_distance = 0;
    for (auto distance : iekf_distances) {
      iekf_mean_distance += distance;
      if (distance > iekf_max_distance)
        iekf_max_distance = distance;
    }
    iekf_mean_distance = iekf_mean_distance / (double) iekf_distances.size();
    std::cout << "IEKF mean distance " << iekf_mean_distance << std::endl;
    std::cout << "IEKF max distance " << iekf_max_distance << std::endl;

    double iekf_mean_angle = 0;
    double iekf_max_angle = 0;
    for (auto angle : iekf_angles) {
      iekf_mean_angle += angle;
      if (angle > iekf_max_angle)
        iekf_max_angle = angle;
    }
    iekf_mean_angle = iekf_mean_angle / (double) iekf_angles.size();
    std::cout << "IEKF mean angle " << 180.0 / M_PI * iekf_mean_angle << std::endl;
    std::cout << "IEKF max angle " << 180.0 / M_PI * iekf_max_angle << std::endl;
  }

  {
    double ukf_mean_distance = 0;
    double ukf_max_distance = 0;
    for (auto distance : ukf_distances) {
      ukf_mean_distance += distance;
      if (distance > ukf_max_distance)
        ukf_max_distance = distance;
    }
    ukf_mean_distance = ukf_mean_distance / (double) ukf_distances.size();
    std::cout << "UKF mean distance " << ukf_mean_distance << std::endl;
    std::cout << "UKF max distance " << ukf_max_distance << std::endl;

    double ukf_mean_angle = 0;
    double ukf_max_angle = 0;
    for (auto angle : ukf_angles) {
      ukf_mean_angle += angle;
      if (angle > ukf_max_angle)
        ukf_max_angle = angle;
    }
    ukf_mean_angle = ukf_mean_angle / (double) ukf_angles.size();
    std::cout << "UKF mean angle " << 180.0 / M_PI * ukf_mean_angle << std::endl;
    std::cout << "UKF max angle " << 180.0 / M_PI * ukf_max_angle << std::endl;
  }

  {
    double pgo_mean_distance = 0;
    double pgo_max_distance = 0;
    for (auto distance : pgo_distances) {
      pgo_mean_distance += distance;
      if (distance > pgo_max_distance)
        pgo_max_distance = distance;
    }
    pgo_mean_distance = pgo_mean_distance / (double) pgo_distances.size();
    std::cout << "PGO mean distance " << pgo_mean_distance << std::endl;
    std::cout << "PGO max distance " << pgo_max_distance << std::endl;

    double pgo_mean_angle = 0;
    double pgo_max_angle = 0;
    for (auto angle : pgo_angles) {
      pgo_mean_angle += angle;
      if (angle > pgo_max_angle)
        pgo_max_angle = angle;
    }
    pgo_mean_angle = pgo_mean_angle / (double) pgo_angles.size();
    std::cout << "PGO mean angle " << 180.0 / M_PI * pgo_mean_angle << std::endl;
    std::cout << "PGO max angle " << 180.0 / M_PI * pgo_max_angle << std::endl;
  }


  // Static results
  // { // APE1
    // {
      // std::cout << "APE1 Position" << std::endl;
      // Eigen::Vector3d ape1_mean_position =
      //   Eigen::Vector3d::Zero();
      // for (auto position : ape1_positions) {
      //   ape1_mean_position += position;
      // }
      // ape1_mean_position = ape1_mean_position / (double)ape1_positions.size();
      // std::cout << ape1_mean_position << std::endl;
      // Eigen::Matrix3d ape1_covariance =
      //   Eigen::Matrix3d::Zero();
      // for (auto position : ape1_positions) {
      //   ape1_covariance += (position - ape1_mean_position) * 
      //     (position - ape1_mean_position).transpose();
      // }
      // std::cout << ape1_covariance << std::endl;
    // }
  //   {
  //     std::cout << "APE1 Attitude" << std::endl;
  //     Eigen::Vector3d ape1_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : ape1_attitudes) {
  //       ape1_mean_attitude += attitude;
  //     }
  //     ape1_mean_attitude = ape1_mean_attitude / (double)ape1_attitudes.size();
  //     std::cout << ape1_mean_attitude << std::endl;
  //     Eigen::Matrix3d ape1_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : ape1_attitudes) {
  //       ape1_covariance += (attitude - ape1_mean_attitude) * 
  //         (attitude - ape1_mean_attitude).transpose();
  //     }
  //     std::cout << ape1_covariance << std::endl;
  //   }
  // }

  // { // APE2
  //   {
  //     std::cout << "APE2 Position" << std::endl;
  //     Eigen::Vector3d ape2_mean_position =
  //       Eigen::Vector3d::Zero();
  //     for (auto position : ape2_positions) {
  //       ape2_mean_position += position;
  //     }
  //     ape2_mean_position = ape2_mean_position / (double)ape2_positions.size();
  //     std::cout << ape2_mean_position << std::endl;
  //     Eigen::Matrix3d ape2_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto position : ape2_positions) {
  //       ape2_covariance += (position - ape2_mean_position) * 
  //         (position - ape2_mean_position).transpose();
  //     }
  //     std::cout << ape2_covariance << std::endl;
  //   }
  //   {
  //     std::cout << "APE2 Attitude" << std::endl;
  //     Eigen::Vector3d ape2_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : ape2_attitudes) {
  //       ape2_mean_attitude += attitude;
  //     }
  //     ape2_mean_attitude = ape2_mean_attitude / (double)ape2_attitudes.size();
  //     std::cout << ape2_mean_attitude << std::endl;
  //     Eigen::Matrix3d ape2_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : ape2_attitudes) {
  //       ape2_covariance += (attitude - ape2_mean_attitude) * 
  //         (attitude - ape2_mean_attitude).transpose();
  //     }
  //     std::cout << ape2_covariance << std::endl;
  //   }
  // }

  // { // EKF
  //   {
  //     std::cout << "EKF Position" << std::endl;
  //     Eigen::Vector3d ekf_mean_position =
  //       Eigen::Vector3d::Zero();
  //     for (auto position : ekf_positions) {
  //       ekf_mean_position += position;
  //     }
  //     ekf_mean_position = ekf_mean_position / (double)ekf_positions.size();
  //     std::cout << ekf_mean_position << std::endl;
  //     Eigen::Matrix3d ekf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto position : ekf_positions) {
  //       ekf_covariance += (position - ekf_mean_position) * 
  //         (position - ekf_mean_position).transpose();
  //     }
  //     std::cout << ekf_covariance << std::endl;
  //   }
  //   {
  //     std::cout << "EKF Attitude" << std::endl;
  //     Eigen::Vector3d ekf_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : ekf_attitudes) {
  //       ekf_mean_attitude += attitude;
  //     }
  //     ekf_mean_attitude = ekf_mean_attitude / (double)ekf_attitudes.size();
  //     std::cout << ekf_mean_attitude << std::endl;
  //     Eigen::Matrix3d ekf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : ekf_attitudes) {
  //       ekf_covariance += (attitude - ekf_mean_attitude) * 
  //         (attitude - ekf_mean_attitude).transpose();
  //     }
  //     std::cout << ekf_covariance << std::endl;
  //   }
  // }

  // { // IEKF
  //   {
  //     std::cout << "IEKF Position" << std::endl;
  //     Eigen::Vector3d iekf_mean_position =
  //       Eigen::Vector3d::Zero();
  //     for (auto position : iekf_positions) {
  //       iekf_mean_position += position;
  //     }
  //     iekf_mean_position = iekf_mean_position / (double)iekf_positions.size();
  //     std::cout << iekf_mean_position << std::endl;
  //     Eigen::Matrix3d iekf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto position : iekf_positions) {
  //       iekf_covariance += (position - iekf_mean_position) * 
  //         (position - iekf_mean_position).transpose();
  //     }
  //     std::cout << iekf_covariance << std::endl;
  //   }
  //   {
  //     std::cout << "IEKF Attitude" << std::endl;
  //     Eigen::Vector3d iekf_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : iekf_attitudes) {
  //       iekf_mean_attitude += attitude;
  //     }
  //     iekf_mean_attitude = iekf_mean_attitude / (double)iekf_attitudes.size();
  //     std::cout << iekf_mean_attitude << std::endl;
  //     Eigen::Matrix3d iekf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : iekf_attitudes) {
  //       iekf_covariance += (attitude - iekf_mean_attitude) * 
  //         (attitude - iekf_mean_attitude).transpose();
  //     }
  //     std::cout << iekf_covariance << std::endl;
  //   }
  // }

  // { // UKF
  //   {
  //     std::cout << "UKF Position" << std::endl;
  //     Eigen::Vector3d ukf_mean_position =
  //       Eigen::Vector3d::Zero();
  //     for (auto position : ukf_positions) {
  //       ukf_mean_position += position;
  //     }
  //     ukf_mean_position = ukf_mean_position / (double)ukf_positions.size();
  //     std::cout << ukf_mean_position << std::endl;
  //     Eigen::Matrix3d ukf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto position : ukf_positions) {
  //       ukf_covariance += (position - ukf_mean_position) * 
  //         (position - ukf_mean_position).transpose();
  //     }
  //     std::cout << ukf_covariance << std::endl;
  //   }
  //   {
  //     std::cout << "UKF Attitude" << std::endl;
  //     Eigen::Vector3d ukf_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : ukf_attitudes) {
  //       ukf_mean_attitude += attitude;
  //     }
  //     ukf_mean_attitude = ukf_mean_attitude / (double)ukf_attitudes.size();
  //     std::cout << ukf_mean_attitude << std::endl;
  //     Eigen::Matrix3d ukf_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : ukf_attitudes) {
  //       ukf_covariance += (attitude - ukf_mean_attitude) * 
  //         (attitude - ukf_mean_attitude).transpose();
  //     }
  //     std::cout << ukf_covariance << std::endl;
  //   }
  // }

  // { // PGO
  //   {
  //     std::cout << "PGO Position" << std::endl;
  //     Eigen::Vector3d pgo_mean_position =
  //       Eigen::Vector3d::Zero();
  //     for (auto position : pgo_positions) {
  //       pgo_mean_position += position;
  //     }
  //     pgo_mean_position = pgo_mean_position / (double)pgo_positions.size();
  //     std::cout << pgo_mean_position << std::endl;
  //     Eigen::Matrix3d pgo_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto position : pgo_positions) {
  //       pgo_covariance += (position - pgo_mean_position) * 
  //         (position - pgo_mean_position).transpose();
  //     }
  //     std::cout << pgo_covariance << std::endl;
  //   }
  //   {
  //     std::cout << "PGO Attitude" << std::endl;
  //     Eigen::Vector3d pgo_mean_attitude =
  //       Eigen::Vector3d::Zero();
  //     for (auto attitude : pgo_attitudes) {
  //       pgo_mean_attitude += attitude;
  //     }
  //     pgo_mean_attitude = pgo_mean_attitude / (double)pgo_attitudes.size();
  //     std::cout << pgo_mean_attitude << std::endl;
  //     Eigen::Matrix3d pgo_covariance =
  //       Eigen::Matrix3d::Zero();
  //     for (auto attitude : pgo_attitudes) {
  //       pgo_covariance += (attitude - pgo_mean_attitude) * 
  //         (attitude - pgo_mean_attitude).transpose();
  //     }
  //     std::cout << pgo_covariance << std::endl;
  //   }
  // }
  // std::cout << trajectory1(0).translation.x << ", "
  //   << trajectory1(0).translation.y << ", "
  //   << trajectory1(0).translation.z << ", "
  //   << trajectory1(0).rotation.w << ", "
  //   << trajectory1(0).rotation.x << ", "
  //   << trajectory1(0).rotation.y << ", "
  //   << trajectory1(0).rotation.z << std::endl;

  // Eigen::Matrix3d vRt;
  // Eigen::Quaterniond vQt;
  // Eigen::Vector3d P;
  // // Read OptiTrack poses
  // rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
  // for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
  //   const tf2_msgs::TFMessage::ConstPtr tf =
  //     bag_it->instantiate<tf2_msgs::TFMessage>();
  //   for (auto tf_it = tf->transforms.begin();
  //     tf_it != tf->transforms.end(); tf_it++) {
  //     std::cout << "OptiTrack: " <<
  //       tf_it->transform.translation.x << ", " <<
  //       tf_it->transform.translation.y << ", " <<
  //       tf_it->transform.translation.z << ", " <<
  //       tf_it->transform.rotation.w << ", " <<
  //       tf_it->transform.rotation.x << ", " <<
  //       tf_it->transform.rotation.y << ", " <<
  //       tf_it->transform.rotation.z << std::endl;
  //     wbag.write("/tf", tf_it->header.stamp, *tf_it);
  //   }
  // }
  // ROS_INFO("OptiTrack' setup complete.");

  // // Data
  // std::vector<std::string> topics;
  // topics.push_back("/loc/vive/light");
  // topics.push_back("/loc/vive/light/");
  // topics.push_back("/loc/vive/imu");
  // topics.push_back("/loc/vive/imu/");
  // size_t counter = 0;
  // rosbag::View view_li(rbag, rosbag::TopicQuery(topics));
  // for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
  //   const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
  //   if (vl != NULL) {
  //     // counter++;
  //     // if (counter < 1400) continue;
  //     // if (counter == 1701) break;
  //     // ROS_INFO("LIGHT");
  //     solver[vl->header.frame_id]->ProcessLight(vl);
  //     // aux_solver[vl->header.frame_id]->ProcessLight(vl);
  //     geometry_msgs::TransformStamped msg;
  //     if (solver[vl->header.frame_id]->GetTransform(msg)) {
  //       std::cout << "Vive: " <<
  //         msg.transform.translation.x << ", " <<
  //         msg.transform.translation.y << ", " <<
  //         msg.transform.translation.z << ", " <<
  //         msg.transform.rotation.w << ", " <<
  //         msg.transform.rotation.x << ", " <<
  //         msg.transform.rotation.y << ", " <<
  //         msg.transform.rotation.z << std::endl;
  //       wbag.write("/tf", vl->header.stamp, msg);
  //     }
  //     if (aux_solver[vl->header.frame_id]->GetTransform(msg)) {
  //       std::cout << "ViveAux: " <<
  //         msg.transform.translation.x << ", " <<
  //         msg.transform.translation.y << ", " <<
  //         msg.transform.translation.z << ", " <<
  //         msg.transform.rotation.w << ", " <<
  //         msg.transform.rotation.x << ", " <<
  //         msg.transform.rotation.y << ", " <<
  //         msg.transform.rotation.z << std::endl;
  //       // wbag.write("/tf", vl->header.stamp, msg);
  //     }
  //     std::cout << std::endl;
  //   }
  //   const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
  //   if (vi != NULL) {
  //     // if (counter < 1400) continue;
  //     // ROS_INFO("IMU");
  //     solver[vi->header.frame_id]->ProcessImu(vi);
  //   }
  // }
  // ROS_INFO("Light read complete.");
  rbag.close();
  wbag.close();


  return 0;
}