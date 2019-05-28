// This package code
#include <hive/vive.h>
#include <hive/vive_general.h>
// #include <hive/hive_solver.h>
// #include <hive/vive_calibrate.h>
// #include <hive/vive_visualization.h>

// Standard C includes
#include <stdio.h>
#include <stdlib.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <nodelet/nodelet.h>
#include <tf2_ros/transform_broadcaster.h>

// Standard C++ includes
#include <iostream>

// Services
#include <hive/ViveConfig.h>

// Messages
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationGeneral.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationLighthouseArray.h>

// Light pose in the vive frame
class HorizontalCost {
public:
  HorizontalCost(hive::ViveLight data,
    Tracker tracker);

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const;
private:
  Tracker tracker_;
  hive::ViveLight data_;
};

// Light pose in the vive frame
class VerticalCost {
public:
  VerticalCost(hive::ViveLight data,
    Tracker tracker);

  template <typename T> bool operator()(const T* const * parameters,
    T * residual) const;
private:
  Tracker tracker_;
  hive::ViveLight data_;
};

HorizontalCost::HorizontalCost(hive::ViveLight data,
  Tracker tracker) {
  data_ = data;
  tracker_ = tracker;
  }

template <typename T> bool HorizontalCost::operator()(const T* const * parameters,
  T * residual) const {
  // Tracker pose
  Eigen::Matrix<T, 3, 1> lPt;
  lPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> lRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

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
    ang = atan(x);

    residual[counter] = T(li_it->angle) - ang;
    counter++;
  }
  return true;
}

VerticalCost::VerticalCost(hive::ViveLight data,
  Tracker tracker) {
  data_ = data;
  tracker_ = tracker;
}

template <typename T> bool VerticalCost::operator()(const T* const * parameters,
  T * residual) const {
  // Tracker pose
  Eigen::Matrix<T, 3, 1> lPt;
  lPt << parameters[0][0],
    parameters[0][1],
    parameters[0][2];
  Eigen::Matrix<T, 3, 3> lRt;
  ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

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
    ang = atan(y);

    residual[counter] = T(li_it->angle) - ang;
    counter++;
  }
  return true;
}


// uint32_t thesensor = 6;

// void LightCallback(const hive::ViveLight::ConstPtr& msg) {
//   // Check the current state of the system
//   std::cout << msg->lighthouse << std::endl;
//   return;
// }

int main(int argc, char ** argv)
{
  // ros::Subscriber sub_light;

  // ros::init(argc, argv, "printer");
  // ros::NodeHandle nh;

  // sub_light = nh.subscribe(TOPIC_HIVE_LIGHT, 1000,
  //   &LightCallback);

  // ros::spin();

  // Read bag with data
  if (argc < 2) {
    std::cout << "Usage: ... hive_calibrator read.bag" << std::endl;
    return -1;
  }
  rosbag::Bag rbag;
  rosbag::View view;
  std::string read_bag(argv[1]);
  rbag.open(read_bag, rosbag::bagmode::Read);

  Calibration calibration;
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

  Tracker tracker = calibration.trackers.begin()->second;
  Lighthouse lighthouse = calibration.lighthouses.begin()->second;
  hive::ViveLight hmsg, vmsg;
  Eigen::Vector3d lPt(0.0,0.0,1.0);
  Eigen::Matrix3d lRt = Eigen::Matrix3d::Identity(3,3);
  size_t N = 20;
  {
    hmsg.lighthouse = lighthouse.serial;
    hmsg.header.frame_id = tracker.serial;
    for (size_t i = 0; i < N; i++) {
      Eigen::Vector3d tPs(tracker.sensors[i].position.x,
        tracker.sensors[i].position.y,
        tracker.sensors[i].position.z);
      Eigen::Vector3d lPs = lRt * tPs + lPt;
      hive::ViveLightSample sample_msg;
      sample_msg.sensor = i;
      sample_msg.angle = atan2(lPs(0),lPs(2));
      hmsg.samples.push_back(sample_msg);
    }
  }
  {
    vmsg.lighthouse = lighthouse.serial;
    vmsg.header.frame_id = tracker.serial;
    for (size_t i = 0; i < N; i++) {
      Eigen::Vector3d tPs(tracker.sensors[i].position.x,
        tracker.sensors[i].position.y,
        tracker.sensors[i].position.z);
      Eigen::Vector3d lPs = lRt * tPs + lPt;
      hive::ViveLightSample sample_msg;
      sample_msg.sensor = i;
      sample_msg.angle = atan2(lPs(1),lPs(2));
      vmsg.samples.push_back(sample_msg);
    }
  }

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // double pose[6] = {-0.9,0.3,1.9,-0.8,0.3,0.5};
  double pose[6] = {-1.3,1.5,2.5,-0.8,1.3,-1.5};

  ceres::DynamicAutoDiffCostFunction<HorizontalCost, 4> * hcost =
    new ceres::DynamicAutoDiffCostFunction<HorizontalCost, 4>
    (new HorizontalCost(hmsg,tracker));
  hcost->AddParameterBlock(6);
  hcost->SetNumResiduals(hmsg.samples.size());
  problem.AddResidualBlock(hcost, NULL, pose);

  ceres::DynamicAutoDiffCostFunction<VerticalCost, 4> * vcost =
    new ceres::DynamicAutoDiffCostFunction<VerticalCost, 4>
    (new VerticalCost(vmsg,tracker));
  vcost->AddParameterBlock(6);
  vcost->SetNumResiduals(vmsg.samples.size());
  problem.AddResidualBlock(vcost, NULL, pose);

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.final_cost <<  " - "
    << pose[0] << ", "
    << pose[1] << ", "
    << pose[2] << ", "
    << pose[3] << ", "
    << pose[4] << ", "
    << pose[5] << std::endl;

  return 0;
}