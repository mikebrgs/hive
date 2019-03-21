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
#include <hive/vive_base.h>
#include <hive/vive_solver.h>
#include <hive/vive_general.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// ROS msgs
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;

// We will use here quaternions
// First parameter is transform from vive to optitrack
// Second parameter is transform from tracker to arrow
struct PoseCostFunctor{
  explicit PoseCostFunctor(TF vive, TF optitrack) :
    vive_(vive), optitrack_(optitrack) {}
  template <typename T> bool operator()(const T* const * parameters,
  T * residual) const {
    // Pose of the tracker in the Vive frame at instant t
    Eigen::Matrix<T, 3, 1> vPt;
    Eigen::Matrix<T, 3, 3> vRt;
    // Pose of the arrow in the optitrack frame at instant t+/-1
    Eigen::Matrix<T, 3, 1> oPa;
    Eigen::Matrix<T, 3, 3> oRa;
    // Transform from vive frame to optitrack frame
    Eigen::Matrix<T, 3, 1> oPv;
    Eigen::Matrix<T, 3, 3> oRv;
    // Transform from tracker frame to arrow frame
    Eigen::Matrix<T, 3, 1> aPt;
    Eigen::Matrix<T, 3, 3> aRt;

    vPt << vive_.transform.translation.x
        << vive_.transform.translation.y
        << vive_.transform.translation.z;
    vRt << Eigen::Quaternion<T>(vive_.transform.rotation.w,
        vive_.transform.rotation.x,
        vive_.transform.rotation.y,
        vive_.transform.rotation.z).toRotationMatrix();
    oPa << optitrack_.transform.translation.x
        << optitrack_.transform.translation.y
        << optitrack_.transform.translation.z;
    oRa << Eigen::Quaternion<T>(optitrack_.transform.rotation.w,
        optitrack_.transform.rotation.x,
        optitrack_.transform.rotation.y,
        optitrack_.transform.rotation.z).toRotationMatrix();
    oPv << parameters[0][0]
        << parameters[0][1]
        << parameters[0][2];
    oRv << Eigen::Quaternion<T>(parameters[0][3],
      parameters[0][4],
      parameters[0][5],
      parameters[0][6]).toRotationMatrix();
    aPt << parameters[1][0]
        << parameters[1][1]
        << parameters[1][2];
    aRt << Eigen::Quaternion<T>(parameters[1][3],
      parameters[1][4],
      parameters[1][5],
      parameters[1][6]).toRotationMatrix();

    // Convert Vive to OptiTrack
    Eigen::Matrix<T, 3, 1> tPa = -aRt.transpose() * aPt;
    Eigen::Matrix<T, 3, 3> tRa = aRt.transpose();

    Eigen::Matrix<T, 3, 1> _vPa = vRt * tPa + vPt;
    Eigen::Matrix<T, 3, 3> _vRa = vRt * tRa;

    Eigen::Matrix<T, 3, 1> _oPa = oRv * _vPa + oPv;
    Eigen::Matrix<T, 3, 3> _oRa = oRv * _vRa;

    residual[0] = _oPa(0) - oPa(0);
    residual[1] = _oPa(1) - oPa(1);
    residual[2] = _oPa(2) - oPa(2);

    return true;
  }
 private:
  TF vive_;
  TF optitrack_;
};

TFs ComputeOffset(TFs optitrack, TFs vive) {
  TF oTv; // Transform from vive to optitrack
  TF aTt; // Transform from tracker to optitrack's arrow

  ceres::Problem problem;

  // TODO put problem here

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.max_solver_time_in_seconds = 1.0;

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  TFs Ts;
  Ts.push_back(oTv);
  Ts.push_back(aTt);
  return Ts;
}

class HiveOffset
{
public:
  HiveOffset(int argc, char ** argv);
  ~HiveOffset();
  void Spin();
private:
  std::string bagname_;
  std::vector<geometry_msgs::TransformStamped> otv_;
  std::vector<geometry_msgs::TransformStamped> vv_;
  Calibration cal_;
  ros::NodeHandle * nh_;
  // BaseMap solver_;
};

HiveOffset::HiveOffset(int argc, char ** argv) {
  ros::init(argc, argv, "hive_offset");
  nh_ = new ros::NodeHandle();


  bagname_ = std::string(argv[1]);
}

HiveOffset::~HiveOffset() {
  
}

void HiveOffset::Spin() {
  rosbag::View view;
  rosbag::Bag rbag;

  Solver * solver;

  rbag.open(bagname_, rosbag::bagmode::Read);
  // Read OptiTrack poses
  rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
  for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
    const tf2_msgs::TFMessage::ConstPtr tf =
      bag_it->instantiate<tf2_msgs::TFMessage>();
    for (auto tf_it = tf->transforms.begin();
      tf_it != tf->transforms.end(); tf_it++) {
      otv_.push_back(*tf_it);
    }
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_BASE_CALIBRATION_FILE, &cal_)) {
    ROS_FATAL("Can't find calibration file.");
    return;
  } else {
    ROS_INFO("Read calibration file.");
  }

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
  ROS_INFO("Trackers' setup complete.");

  // Temporary solver - Easily changed
  solver = new BaseSolve(cal_.environment,
    cal_.trackers.begin()->second);

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    solver->ProcessLight(vl);
    geometry_msgs::TransformStamped msg;
    solver->GetTransform(msg);
    vv_.push_back(msg);
  }
  ROS_INFO("Data processment complete.");

  rbag.close();
  return;
}

int main(int argc, char ** argv) {
  // Read bag with data
  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << "name_of_write_bag.bag" << std::endl;
    return -1;
  }

  // Initialization (including name of data file)
  HiveOffset ho(argc, argv);

  ho.Spin();

  return 0;
}