/**
 * @version 1.0
 * @author Miguel Rego Borges
 * Instituto Superior Tecnico - University of Lisbon
 * Purpose: calculate offset between vive's frame and optitrack's
*/

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Standard C includes
#include <stdlib.h>
#include <stdio.h>

// Standard C++ includes
#include <iostream>
#include <string>

// Hive includes
#include <hive/vive.h>
// #include <hive/vive_base.h> // TODO Can't import because it has a main
#include <hive/vive_general.h>

// ROS msgs
#include <geometry_msgs/TransformStamped.h>
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>

class HiveOffset
{
public:
  HiveOffset(int argc, char ** argv);
  ~HiveOffset();
  void Spin();
private:
  std::string bagname_;
  std::vector<geometry_msgs::TransformStamped> otv_;
  Calibration cal_;
  // BaseMap solver_;
};

HiveOffset::HiveOffset(int argc, char ** argv) {
  ros::init(argc, argv, "hive_offset");
  ros::NodeHandle nh;


  bagname_ = std::string(argv[1]);
}

HiveOffset::~HiveOffset() {
  
}

void HiveOffset::Spin() {
  rosbag::View view;
  rosbag::Bag rbag;

  rbag.open(bagname_, rosbag::bagmode::Read);

  // Read OptiTrack poses
  rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
  for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
    const geometry_msgs::TransformStamped::ConstPtr tf =
      bag_it->instantiate<geometry_msgs::TransformStamped>();
    otv_.push_back(*tf);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal_.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal_.SetTrackers(*vt);
  }
  for (auto tr_it = cal_.trackers.begin();
    tr_it != cal_.trackers.end(); tr_it++) {
    // solver_[tr_it->first] = BaseSolve(cal_.environment,
    //   tr_it->second);
  }
  ROS_INFO("Trackers' setup complete.");

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    // solver_[vl->header.frame_id].ProcessLight(vl);
    geometry_msgs::TransformStamped msg;
  }

  rbag.close();
  return;
}

int main(int argc, char ** argv)
{
  // Initializations
  HiveOffset ho(argc, argv);

  // Read bag with data
  if (argc <= 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << "name_of_write_bag.bag" << std::endl;
    return -1;
  }

  ho.Spin();

  return 0;
}