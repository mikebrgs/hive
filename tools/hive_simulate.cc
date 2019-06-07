// Includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Hive imports
#include <hive/hive_solver.h>
#include <hive/vive_filter.h>
#include <hive/vive_pgo.h>
#include <hive/vive_general.h>

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
    std::map<std::string, Lighthouse> lh_specs);
  ~Trajectory();
  void Update(double dt);
  sensor_msgs::Imu GetImu();
  hive::ViveLight GetLight();
  geometry_msgs::TransformStamped GetTransform();
  void PrintState();
};

Trajectory::Trajectory(TrajectoryFunction tfun,
    Tracker tracker,
    std::map<std::string, Transform> lh_poses,
    std::map<std::string, Lighthouse> lh_specs) {
  time_ = ros::Time(0);
  axis_ = HORIZONTAL;
  tfun_ = tfun;
  this_state_.position = Eigen::Vector3d(
    tfun(0).translation.x,
    tfun(0).translation.y,
    tfun(0).translation.z);
  this_state_.rotation = Eigen::Quaterniond(
    tfun(0).rotation.w,
    tfun(0).rotation.x,
    tfun(0).rotation.y,
    tfun(0).rotation.z);
  this_state_.velocity = Eigen::Vector3d::Zero();
  this_state_.acceleration = Eigen::Vector3d::Zero();
  this_state_.angular = Eigen::Vector3d::Zero();
  tracker_ = tracker;
  lh_poses_ = lh_poses;
  lh_specs_ = lh_specs;
  lh_pointer_ = lh_poses_.begin();
  precision_ = 1e-6;
  acc_distribution_ = std::normal_distribution<double>(0,1e-2);
  gyr_distribution_ = std::normal_distribution<double>(0,1e-2);
  lig_distribution_ = std::normal_distribution<double>(0,1e-5);
  light_used_ = false;
  return;
}

Trajectory::~Trajectory() {
  // Do nothing
  return;
}

bool comparator(Triplet a, Triplet b) {
  return std::get<2>(a) < std::get<2>(b);
}

hive::ViveLight Trajectory::GetLight() {
  hive::ViveLight msg;
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

    data.push_back(std::make_tuple(
      sensor.first,
      angle + lig_distribution_(generator_),
      dproduct));
  }

  std::sort(data.begin(), data.end(), comparator);

  for (size_t i = 0; i < 9; i++) {
    if (std::get<2>(data[i]) > 0) break;
    hive::ViveLightSample sample_msg;
    sample_msg.sensor = std::get<0>(data[i]);
    sample_msg.angle = std::get<1>(data[i]);
    msg.samples.push_back(sample_msg);
  }

  msg.lighthouse = lh_pointer_->first;
  msg.header.frame_id = tracker_.serial;
  msg.header.stamp = time_;
  msg.axis = static_cast<uint8_t>(axis_);

  light_used_ = true;

  return msg;
}

sensor_msgs::Imu Trajectory::GetImu() {
  sensor_msgs::Imu msg;

  msg.linear_acceleration.x = this_state_.acceleration(0)
    + acc_distribution_(generator_);
  msg.linear_acceleration.y = this_state_.acceleration(1)
    + acc_distribution_(generator_);
  msg.linear_acceleration.z = this_state_.acceleration(2)
    + acc_distribution_(generator_);

  msg.angular_velocity.x = this_state_.angular(0)
    + gyr_distribution_(generator_);
  msg.angular_velocity.y = this_state_.angular(1)
    + gyr_distribution_(generator_);
  msg.angular_velocity.z = this_state_.angular(2)
    + gyr_distribution_(generator_);

  msg.header.frame_id = tracker_.serial;
  msg.header.stamp = time_;

  return msg;
}

void Trajectory::Update(double dt) {
  time_ = time_ + ros::Duration(dt);

  // Poses
  geometry_msgs::Transform msg3 = tfun_(time_.toSec() - 2.0 * precision_);
  geometry_msgs::Transform msg2 = tfun_(time_.toSec() - 1.0 * precision_);
  geometry_msgs::Transform msg1 = tfun_(time_.toSec() - 0.0 * precision_);

  // Positions
  Eigen::Vector3d vPi_1(msg1.translation.x,
    msg1.translation.y,
    msg1.translation.z);
  Eigen::Vector3d vPi_2(msg2.translation.x,
    msg2.translation.y,
    msg2.translation.z);
  Eigen::Vector3d vPi_3(msg3.translation.x,
    msg3.translation.y,
    msg3.translation.z);

  // Linear velocities
  Eigen::Vector3d vVi_1 = (vPi_1 - vPi_2) / precision_;
  Eigen::Vector3d vVi_2 = (vPi_2 - vPi_3) / precision_;

  // Accelerations
  Eigen::Vector3d vAi = (vVi_1 - vVi_2) / precision_;

  // Orientations
  Eigen::Vector4d vQi_1(msg1.rotation.w,
    msg1.rotation.x,
    msg1.rotation.y,
    msg1.rotation.z);
  Eigen::Vector4d vQi_2(msg2.rotation.w,
    msg2.rotation.x,
    msg2.rotation.y,
    msg2.rotation.z);
  Eigen::Vector4d vDQi = (vQi_1 - vQi_2) / precision_;

  //0.5 * Omega matrix
  Eigen::Matrix<double, 4, 3> A;
  A(0,0) = -vQi_1(1);
  A(0,1) = -vQi_1(2);
  A(0,2) = -vQi_1(3);
  A(1,0) = vQi_1(0);
  A(1,1) = -vQi_1(3);
  A(1,2) = vQi_1(2);
  A(2,0) = vQi_1(3);
  A(2,1) = vQi_1(0);
  A(2,2) = -vQi_1(1);
  A(3,0) = -vQi_1(2);
  A(3,1) = vQi_1(1);
  A(3,2) = vQi_1(0);
  A = 0.5 * A;

  // Angular velocity
  Eigen::Vector3d vWi = (A.transpose() * A).inverse() * A.transpose() * vDQi;

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
  this_state_.acceleration = vAi;
  this_state_.rotation = Eigen::Quaterniond(
    vQi_1(0),
    vQi_1(1),
    vQi_1(2),
    vQi_1(3));
  this_state_.angular = vWi;
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
  std::cout << "P: " << this_state_.acceleration(0) << ", "
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


// Main function
int main(int argc, char ** argv) {
  // Data
  Calibration calibration;
  std::map<std::string, Solver*> solver;
  std::map<std::string, Solver*> aux_solver;

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
  }
  for (auto tracker : calibration.trackers) {
    // Aux solver
    aux_solver[tracker.first] = new HiveSolver(calibration.trackers[tracker.first],
      calibration.lighthouses,
      calibration.environment,
      true);
    // APE
    solver[tracker.first] = new HiveSolver(calibration.trackers[tracker.first],
      calibration.lighthouses,
      calibration.environment,
      true);
    // EKF
    // solver[tracker.first] = new ViveFilter(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   1e-1, 1e-6, true, filter::ekf);
    // IEKF
    // solver[tracker.first] = new ViveFilter(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   1e-1, 1e-6, true, filter::iekf);
    // UKF
    // solver[tracker.first] = new ViveFilter(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   1.0e0, 1e-6, true, filter::ukf);
    // PGO
    // solver[tracker.first] = new PoseGraph(calibration.environment,
    //   calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   4, 7e-4, 1e0, true);
  }
  ROS_INFO("Trackers' setup complete.");

  Trajectory tr(&trajectory1,
    calibration.trackers.begin()->second,
    calibration.environment.lighthouses,
    calibration.lighthouses);

  tr.PrintState();
  tr.Update(0.005);

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