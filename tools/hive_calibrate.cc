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
  ROS_INFO("Reading JSON file.");
  jp.GetBody(&calibration);

  ViveCalibrate calibrator(calibration, true);

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    calibrator.Update(vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    calibrator.Update(vt);
  }
  ROS_INFO("Trackers' setup complete.");

  // Light data
  size_t counter = 0;
  std::vector<std::string> imu_topics;
  imu_topics.push_back("/loc/vive/imu");
  imu_topics.push_back("/loc/vive/imu/");
  rosbag::View view_imu(rbag, rosbag::TopicQuery(imu_topics));
  for (auto bag_it = view_imu.begin(); bag_it != view_imu.end(); bag_it++) {
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    calibrator.AddImu(vi);
    ROS_INFO("ADDED IMU");
    counter++;
    if (counter >= 20) break;
  }
  ROS_INFO("Imu read complete.");

  // Light data
  counter = 0;
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    counter++;
    // if (counter < 100) continue;
    if (counter > 50) break;
    calibrator.AddLight(vl);
  }
  ROS_INFO("Light read complete.");
  rbag.close();

  calibrator.Solve();
  ViveUtils::WriteConfig(HIVE_CALIBRATION_FILE,
    calibrator.GetCalibration());

  return 0;
}