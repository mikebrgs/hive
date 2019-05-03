// Includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Hive imports
#include <hive/hive_solver.h>
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
  std::map<std::string, Solver*> solver;

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
    solver[tracker.first] = new HiveSolver(calibration.trackers[tracker.first],
      calibration.lighthouses,
      calibration.environment,
      true);
  }
  ROS_INFO("Trackers' setup complete.");


    // Read OptiTrack poses
    rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
    for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
      const tf2_msgs::TFMessage::ConstPtr tf =
        bag_it->instantiate<tf2_msgs::TFMessage>();
      for (auto tf_it = tf->transforms.begin();
        tf_it != tf->transforms.end(); tf_it++) {
        std::cout << "OptiPose" << std::endl;
        wbag.write("/tf", tf_it->header.stamp, *tf_it);
      }
    }

  // Light data
  size_t counter = 0;
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    solver[vl->header.frame_id]->ProcessLight(vl);
    geometry_msgs::TransformStamped msg;
    if (solver[vl->header.frame_id]->GetTransform(msg))
      std::cout << "VivePose" << std::endl;
      wbag.write("/tf", vl->header.stamp, msg);
    counter++;
  }
  ROS_INFO("Light read complete.");
  rbag.close();
  wbag.close();


  return 0;
}