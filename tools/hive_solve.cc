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
    // APE1
    // solver[tracker.first] = new HiveSolver(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   false);
    // APE2
    // solver[tracker.first] = new HiveSolver(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   true);
    // EKF
    solver[tracker.first] = new ViveFilter(calibration.trackers[tracker.first],
      calibration.lighthouses,
      calibration.environment,
      1e0, 1e-6, true, filter::ekf);
    // IEKF
    // solver[tracker.first] = new ViveFilter(calibration.trackers[tracker.first],
    //   calibration.lighthouses,
    //   calibration.environment,
    //   1e0, 1e-6, true, filter::iekf);
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

  Eigen::Matrix3d vRt;
  Eigen::Quaterniond vQt;
  Eigen::Vector3d P;
    // Read OptiTrack poses
    rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
    for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
      const tf2_msgs::TFMessage::ConstPtr tf =
        bag_it->instantiate<tf2_msgs::TFMessage>();
      for (auto tf_it = tf->transforms.begin();
        tf_it != tf->transforms.end(); tf_it++) {
        std::cout << "OptiTrack: " <<
          tf_it->transform.translation.x << ", " <<
          tf_it->transform.translation.y << ", " <<
          tf_it->transform.translation.z << ", " <<
          tf_it->transform.rotation.w << ", " <<
          tf_it->transform.rotation.x << ", " <<
          tf_it->transform.rotation.y << ", " <<
          tf_it->transform.rotation.z << std::endl;
        wbag.write("/tf", tf_it->header.stamp, *tf_it);
      }
    }
  ROS_INFO("OptiTrack' setup complete.");

  // Data
  std::vector<std::string> topics;
  topics.push_back("/loc/vive/light");
  topics.push_back("/loc/vive/light/");
  topics.push_back("/loc/vive/imu");
  topics.push_back("/loc/vive/imu/");
  size_t counter = 0;
  rosbag::View view_li(rbag, rosbag::TopicQuery(topics));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    if (vl != NULL) {
      // counter++;
      // if (counter < 1400) continue;
      // if (counter == 1701) break;
      // ROS_INFO("LIGHT");
      solver[vl->header.frame_id]->ProcessLight(vl);
      // aux_solver[vl->header.frame_id]->ProcessLight(vl);
      geometry_msgs::TransformStamped msg;
      if (solver[vl->header.frame_id]->GetTransform(msg)) {
        std::cout << "Vive: " <<
          msg.transform.translation.x << ", " <<
          msg.transform.translation.y << ", " <<
          msg.transform.translation.z << ", " <<
          msg.transform.rotation.w << ", " <<
          msg.transform.rotation.x << ", " <<
          msg.transform.rotation.y << ", " <<
          msg.transform.rotation.z << std::endl;
        wbag.write("/tf", vl->header.stamp, msg);
      }
      if (aux_solver[vl->header.frame_id]->GetTransform(msg)) {
        std::cout << "ViveAux: " <<
          msg.transform.translation.x << ", " <<
          msg.transform.translation.y << ", " <<
          msg.transform.translation.z << ", " <<
          msg.transform.rotation.w << ", " <<
          msg.transform.rotation.x << ", " <<
          msg.transform.rotation.y << ", " <<
          msg.transform.rotation.z << std::endl;
        // wbag.write("/tf", vl->header.stamp, msg);
      }
      std::cout << std::endl;
    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      // if (counter < 1400) continue;
      // ROS_INFO("IMU");
      solver[vi->header.frame_id]->ProcessImu(vi);
    }
  }
  ROS_INFO("Light read complete.");
  rbag.close();
  wbag.close();


  return 0;
}