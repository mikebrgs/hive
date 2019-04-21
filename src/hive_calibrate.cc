// Includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Hive imports
#include <hive/hive_calibrator.h>
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
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <string>

// Hive includes

// Main function
int main(int argc, char ** argv) {
  // Data
  Calibration calibration;

  // Read bag with data
  if (argc < 2) {
    std::cout << "Usage: ... hive_calibrator name_of_read_bag.bag" << std::endl;
    return -1;
  }
  rosbag::Bag rbag;
  rosbag::View view;
  std::string read_bag(argv[1]);
  rbag.open(read_bag, rosbag::bagmode::Read);

  // Start JSON parser
  JsonParser jp = JsonParser(HIVE_CONFIG_FILE);
  // Get current calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &calibration)) {
    jp.GetCalibration(&calibration);
    ROS_WARN("Reading JSON file.");
  } else {
    jp.GetBody(&calibration);
    ROS_INFO("Read calibration file.");
  }

  ViveCalibrate calibrator(calibration, false);

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    // calibration.SetLighthouses(*vl);
    calibrator.Update(vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    // calibration.SetTrackers(*vt);
    calibrator.Update(vt);
  }
  ROS_INFO("Trackers' setup complete.");


  // Light data
  // size_t counter = 0;
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    calibrator.AddLight(vl);
  }
  rbag.close();

  calibrator.Solve();
  ViveUtils::WriteConfig(HIVE_CALIBRATION_FILE,
    calibrator.GetCalibration());

  // ros::shutdown();
  return 0;
}