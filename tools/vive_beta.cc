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

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
// ROS messages
#include <hive/ViveLight.h>
#include <hive/HiveData.h>

// Hive includes
#include "hive/vive_solve.h"
#include "hive/vive.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "beta");
  ros::NodeHandle nh;

  rosbag::Bag rbag, wbag;
  rbag.open("/home/mikebrgs/CurrentWork/thesis/vive1/catkin_ws/src/hive/data/vivebeta3.bag",
    rosbag::bagmode::Read);
  rosbag::View view;
  wbag.open("hive_data.bag",
    rosbag::bagmode::Write);

  // Declarations
  std::mutex mtx;
  std::string last_lh;
  LightData observations;
  SolvedPose ref_pose;
  Lighthouse lh;
  Extrinsics tr;
  Calibration cal;

  // Initializations
  ref_pose.transform[0] = 0.0;
  ref_pose.transform[1] = 0.0;
  ref_pose.transform[2] = 1.0;
  ref_pose.transform[3] = 0.0;
  ref_pose.transform[4] = 0.0;
  ref_pose.transform[5] = 0.0;

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal.SetTrackers(*vt);
    wbag.write("/trackers", ros::Time::now() ,vt);
  }

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
    wbag.write("/lighthouses", ros::Time::now() ,vl);
  }

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    observations[vl->lighthouse].axis[vl->axis].lights.clear();
    for (auto li_it = vl->samples.begin();
      li_it != vl->samples.end(); li_it++) {
      // Filling in the data structure
      observations[vl->lighthouse].lighthouse = vl->lighthouse;
      observations[vl->lighthouse].axis[vl->axis].stamp = vl->header.stamp;
      Light light;
      light.sensor_id = li_it->sensor;
      light.timecode = li_it->timecode;
      light.angle = li_it->angle;
      light.length = li_it->length;
      observations[vl->lighthouse].axis[vl->axis].lights.push_back(light);
    }

    tr.size = ViveUtils::ConvertExtrinsics(cal.trackers[vl->header.frame_id],
      tr.positions);
    // If the data is available
    if (observations[vl->lighthouse].axis[0].lights.size() != 0 &&
      observations[vl->lighthouse].axis[1].lights.size() != 0) {
      // Compute pose of the tracker
      // SolvedPose * pose = new SolvedPose();
      SolvedPose pose;
      for (size_t i = 0; i < 6; i++) pose.transform[i] = ref_pose.transform[i];
      // pose_vector.push_back(pose);
      if (ComputeTransform(observations[vl->lighthouse],
        &pose,
        &last_lh,
        &tr,
        &mtx,
        &lh)) {
        hive::HiveData hdata;
        hdata.header.stamp = vl->header.stamp;
        hdata.position.x = pose.transform[0];
        hdata.position.y = pose.transform[1];
        hdata.position.z = pose.transform[2];
        hdata.axisangle.x = pose.transform[3];
        hdata.axisangle.y = pose.transform[4];
        hdata.axisangle.z = pose.transform[5];
        hdata.lighthouse = vl->lighthouse;
        hdata.tracker = vl->header.frame_id;
        hdata.axis = vl->axis;
        hdata.samples = vl->samples;
        wbag.write("/tdata", ros::Time::now(), hdata);
      }
    }
  }
  rbag.close();
  wbag.close();

  return 0;
}