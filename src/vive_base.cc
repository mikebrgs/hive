// C standard includes
#include <stdlib.h>

// C++ standard includes
#include <iostream>
#include <mutex>

// Ceres and logging
#include <ceres/ceres.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>
// ROS messages
#include <hive/ViveLight.h>
#include <hive/HiveData.h>

// Hive includes
// #include "hive/vive_solve.h" // - Create new solve
#include <hive/vive_general.h>
#include "hive/vive.h"

int main(int argc, char ** argv)
{
  // Data
  Calibration cal;

  ros::init(argc, argv, "hive_baseline_solver");
  ros::NodeHandle nh;

  // Read bag with data
  if (argc < 2) {
    std::cout << "Usage: ... hive_base name_of_the_bag.bag"
      << std::endl;
  }
  rosbag::Bag rbag;
  rosbag::View view;
  std::string bag_name(argv[1]);
  rbag.open(bag_name);

  // Get current calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
    ROS_FATAL("Can't find calibration file.");
    return -1;
  } else {
    ROS_INFO("Read calibration file.");
  }


  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal.SetTrackers(*vt);
    std::cout << "Tracker" << std::endl;
  }

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
    std::cout << "Lighthouse" << std::endl;
  }

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    // do something
    // std::cout << "Light" << std::endl;
  }

  ros::shutdown();
  return 0;
}