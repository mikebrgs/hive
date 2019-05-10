// ROS includes
#include <ros/ros.h>

// Messages
#include <hive/ViveLight.h>
#include <hive/ViveLightSample.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationLighthouseArray.h>
#include <hive/ViveCalibrationTrackerArray2.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationGeneral.h>
#include <tf2_msgs/TFMessage.h>

// Hive includes
#include <hive/vive.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <hive/hive_solver.h>
#include <hive/vive_general.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL C++ includes
#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <cstdio>
#include <vector>
#include <map>

int main(int argc, char ** argv) {

  if (argc < 3) {
    std::cout << "rosrun hive hive_print_offset"
      << "<offset_cal>.bag <data>.bag" << std::endl;
      return -1;
  }

  // Calibration structure
  Calibration calibration;
  // Solver structure
  std::map<std::string, Solver*> solver;

  rosbag::View view;
  rosbag::Bag data_bag, offset_bag;
  offset_bag.open(argv[1], rosbag::bagmode::Read);
  data_bag.open(argv[2], rosbag::bagmode::Read);

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &calibration)) {
    ROS_FATAL("Can't find calibration file.");
    return false;
  } else {
    ROS_INFO("Read calibration file.");
  }

  // Read Offset poses
  geometry_msgs::TransformStamped aMt, oMv;
  Eigen::Matrix3d aRt, oRv;
  Eigen::Vector3d aPt, oPv;
  //
  rosbag::View view_offset(offset_bag, rosbag::TopicQuery("/offset"));
  for (auto bag_it = view_offset.begin(); bag_it != view_offset.end(); bag_it++) {
    const geometry_msgs::TransformStamped::ConstPtr tf =
      bag_it->instantiate<geometry_msgs::TransformStamped>();
      // Save small offset
      if (tf->header.frame_id == "arrow" &&
        tf->child_frame_id == "tracker") {
        aMt = *tf;
        aPt = Eigen::Vector3d(aMt.transform.translation.x,
          aMt.transform.translation.y,
          aMt.transform.translation.z);
        aRt = Eigen::Quaterniond(aMt.transform.rotation.w,
          aMt.transform.rotation.x,
          aMt.transform.rotation.y,
          aMt.transform.rotation.z).toRotationMatrix();
      }
      // Save big offset
      if (tf->header.frame_id == "optitrack" &&
        tf->child_frame_id == "vive") {
        oMv = *tf;
        oPv = Eigen::Vector3d(oMv.transform.translation.x,
          oMv.transform.translation.y,
          oMv.transform.translation.z);
        oRv = Eigen::Quaterniond(oMv.transform.rotation.w,
          oMv.transform.rotation.x,
          oMv.transform.rotation.y,
          oMv.transform.rotation.z).toRotationMatrix();
      }
  }
  ROS_INFO("Offset setup complete.");
  offset_bag.close();

  // Lighthouses
  rosbag::View view_lh(data_bag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    calibration.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(data_bag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    calibration.SetTrackers(*vt);
    for (auto tracker : calibration.trackers) {
      solver[tracker.first] = new HiveSolver(tracker.second,
        calibration.lighthouses,
        calibration.environment,
        true);
    }
  }
  ROS_INFO("Trackers' setup complete.");

  // Saving poses
  geometry_msgs::TransformStamped opti_msg, vive_msg;
  ros::Time opti_time, vive_time;
  Eigen::Matrix3d vRt, oRa;
  Eigen::Vector3d vPt, oPa;

  // Light data
  std::vector<std::string> topics;
  topics.push_back("/loc/vive/light");
  // topics.push_back("/loc/vive/imu");
  topics.push_back("/tf");
  topics.push_back("tf");
  size_t counter = 0;
  rosbag::View view_li(data_bag, rosbag::TopicQuery(topics));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    // Light data
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    if (vl != NULL) {
      solver[vl->header.frame_id]->ProcessLight(vl);
      if (solver[vl->header.frame_id]->GetTransform(vive_msg)) {
        vPt = Eigen::Vector3d(vive_msg.transform.translation.x,
          vive_msg.transform.translation.y,
          vive_msg.transform.translation.z);
        vRt = Eigen::Quaterniond(vive_msg.transform.rotation.w,
          vive_msg.transform.rotation.x,
          vive_msg.transform.rotation.y,
          vive_msg.transform.rotation.z).toRotationMatrix();
        vive_time = vive_msg.header.stamp;
      } else {
        continue;
      }
    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      // Not active
    }
    const tf2_msgs::TFMessage::ConstPtr vt = bag_it->instantiate<tf2_msgs::TFMessage>();
    if (vt != NULL) {
      for (auto tf_it = vt->transforms.begin();
        tf_it != vt->transforms.end(); tf_it++) {
        opti_msg = *tf_it;
        oPa = Eigen::Vector3d(opti_msg.transform.translation.x,
          opti_msg.transform.translation.y,
          opti_msg.transform.translation.z);
        oRa = Eigen::Quaterniond(opti_msg.transform.rotation.w,
          opti_msg.transform.rotation.x,
          opti_msg.transform.rotation.y,
          opti_msg.transform.rotation.z).toRotationMatrix();
        opti_time = tf_it->header.stamp;
      }
    }
    // Transform
    if (((opti_time - vive_time).toSec()) < 0.05) {
      Eigen::Matrix3d est_aRt = oRa.transpose() * oRv * vRt;
      Eigen::Vector3d est_aPt = oRa.transpose() * (
        oRv * vPt + oPv) + (-oRa.transpose() * oPa);
      Eigen::AngleAxisd dA(est_aRt.transpose() * aRt);
      Eigen::AngleAxisd oAa(oRa);
      Eigen::Vector3d oVa = oAa.axis() * oAa.angle();
      Eigen::AngleAxisd vAt(vRt);
      Eigen::Vector3d vVt = vAt.axis() * vAt.angle();
      Eigen::Vector3d dP = est_aPt - aPt;
      std::cout << dA.angle();
       // << " | " << oVa.transpose()
       // << "   " << vVt.transpose() << std::endl;
      std::cout << " - " << dP.norm() << std::endl;
     }
  }


  data_bag.close();

  return 0;
}