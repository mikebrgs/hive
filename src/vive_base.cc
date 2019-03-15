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

namespace base{
  double start_pose[6] = {0, 0, 1, 0, 0, 0};
}

class BaseSolve {
public:
  BaseSolve();
  ~BaseSolve();
  bool Initialize(Environment const& environment,
    Tracker const& tracker);
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  bool GetTransform(geometry_msgs::TransformStamped &msg);
};

BaseSolve::BaseSolve() {
  // Do nothing
}

BaseSolve::~BaseSolve() {
  // Do nothing
}

void BaseSolve::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) {
    return;
  }

  // // Check if this is a new lighthouse
  // if (poses_.find(msg->lighthouse) == poses_.end()) {
  //   // Set structures
  //   observations_[msg->lighthouse].lighthouse = msg->lighthouse;
  // }

  // // Clear the axis with old data - should only do this if the new data is good
  // observations_[msg->lighthouse].axis[msg->axis].lights.clear();

  // // Iterate all sweep data
  // // std::cout << msg->lighthouse << "-" << static_cast<int>(msg->axis) << " ";
  // for (std::vector<hive::ViveLightSample>::const_iterator li_it = msg->samples.begin();
  //   li_it != msg->samples.end(); li_it++) {
  //   // Detect non-sense data
  //   if (li_it->sensor == -1
  //     || li_it->angle > M_PI/3
  //     || li_it->angle < -M_PI/3) {
  //     continue;
  //   }

  //   // New Light
  //   Light light;
  //   light.sensor_id = li_it->sensor;
  //   light.angle = li_it->angle;
  //   light.timecode = li_it->timecode;
  //   light.length = li_it->length;

  //   // std::cout << li_it->sensor << ":" << li_it->angle << " ";

  //   // Add data to the axis
  //   observations_[msg->lighthouse].axis[msg->axis].lights.push_back(light);
  //   observations_[msg->lighthouse].axis[msg->axis].stamp = msg->header.stamp;
  // }
  // // std::cout << std::endl;

  // // Remove old data
  // for (LightData::iterator lh_it = observations_.begin();
  //   lh_it != observations_.end(); lh_it++) {
  //   for (std::map<uint8_t, LightVecStamped>::iterator ax_it = lh_it->second.axis.begin();
  //     ax_it != lh_it->second.axis.end(); ax_it++) {
  //     ros::Duration elapsed = ax_it->second.stamp - msg->header.stamp;
  //     if (elapsed.toNSec() >= 50e6)
  //       ax_it->second.lights.clear();
  //   }
  // }

  // ros::Duration elapsed = observations_[msg->lighthouse].axis[VERTICAL].stamp -
  //   observations_[msg->lighthouse].axis[HORIZONTAL].stamp;
  // if (observations_[msg->lighthouse].axis[HORIZONTAL].lights.size() > 3
  //   && observations_[msg->lighthouse].axis[VERTICAL].lights.size() > 3) {
  //   ComputeTransformBundle(observations_,
  //     &tracker_pose_,
  //     &extrinsics_,
  //     &environment_,
  //     solveMutex_,
  //     &lh_extrinsics_);
  // }
  return;
}


typedef std::map<std::string, BaseSolve> BaseMap;

int main(int argc, char ** argv)
{
  // Data
  Calibration cal;
  BaseMap solver;


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
  }
  ROS_INFO("Trackers' setup complete.");

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    // do something
  }

  ros::shutdown();
  return 0;
}