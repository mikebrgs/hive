// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Standard C includes
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Standard C++ includes
#include <iostream>
#include <string>
#include <random>

// Hive includes
#include <hive/vive.h>
// #include <hive/vive_cost.h>
#include <hive/hive_solver.h>
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

// The best way to average poses -- including orientation
class OrientationAverageCost{
public:
  // Constructor
  OrientationAverageCost(geometry_msgs::Quaternion sample, double weight);
  // CEres operator
  template <typename T>
  bool operator()(const T* const average, T * residual) const;
private:
  geometry_msgs::Quaternion sample_;
  double weight_;
};

OrientationAverageCost::OrientationAverageCost(geometry_msgs::Quaternion sample,
  double weight) {
  sample_ = sample;
  weight_ = weight;
  return;
}

template <typename T>
bool OrientationAverageCost::operator()(const T* const average,
  T * residual) const {
  // Converting sample
  Eigen::Quaternion<T> Qsample(T(sample_.w),
    T(sample_.x),
    T(sample_.y),
    T(sample_.z));
  Eigen::Matrix<T, 3, 3> Rsample = Qsample.toRotationMatrix();

  // Converting parameter
  Eigen::Matrix<T, 3, 3> Raverage;
  // std::cout << average[0] << ", "
  //   << average[1] << ", "
  //   << average[2] << std::endl;
  ceres::AngleAxisToRotationMatrix(average, Raverage.data());
  // std::cout << Raverage(0,0) << ", " << Raverage(0,1) << ", " << Raverage(0,2) << ", "
  //   << Raverage(1,0) << ", " << Raverage(1,1) << ", " << Raverage(1,2) << ", "
  //   << Raverage(2,0) << ", " << Raverage(2,1) << ", " << Raverage(2,2) << std::endl;

  // Difference
  Eigen::Matrix<T, 3, 3> Rdiff = Rsample * Raverage.transpose();

  // Orientation cost
  T AAdiff[3];
  ceres::RotationMatrixToAngleAxis(Rdiff.data(), AAdiff);
  residual[0] = weight_ * sqrt(AAdiff[0] * AAdiff[0] +
    AAdiff[1] * AAdiff[1] +
    AAdiff[2] * AAdiff[2]);

  return true;
}

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
  ros::Time opti_time(0), vive_time(0);
  Eigen::Matrix3d vRt, oRa;
  Eigen::Vector3d vPt, oPa;
  // Interpolation
  Eigen::Matrix3d prev_oRa, next_oRa;
  Eigen::Vector3d prev_oPa, next_oPa;
  ros::Time prev_opti_time, next_opti_time;

  // Light data
  std::vector<std::string> topics;
  topics.push_back("/loc/vive/light");
  // topics.push_back("/loc/vive/imu");
  topics.push_back("/tf");
  topics.push_back("tf");
  // Position
  Eigen::Vector3d sP = Eigen::Vector3d::Zero();
  double sD = 0.0;
  double mD = 0.0;
  // Attitude
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  double dA[3];
  dA[0] = 0.0;
  dA[1] = 0.0;
  dA[2] = 0.0;
  double sA = 0.0;
  double mA = 0.0;
  // Counter
  double pose_counter = 0;
  // Initializations
  bool vive_init = false, opti_init = false;
  bool prev_opti = false, next_opti = false;
  // Scan bag
  rosbag::View view_li(data_bag, rosbag::TopicQuery(topics));

  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    // Light data
    {
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
    }
    // IMU data
    {
      const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
      if (vi != NULL) {
        // Not active
      }
    }
    // Offset TF2 data
    {
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
    }
    // Regular TF data
    {
      const geometry_msgs::TransformStamped::ConstPtr vt =
        bag_it->instantiate<geometry_msgs::TransformStamped>();
      // Vive transform
      if (vt != NULL && vt->header.frame_id == "vive") {
        vPt = Eigen::Vector3d(vt->transform.translation.x,
          vt->transform.translation.y,
          vt->transform.translation.z);
        vRt = Eigen::Quaterniond(vt->transform.rotation.w,
          vt->transform.rotation.x,
          vt->transform.rotation.y,
          vt->transform.rotation.z).toRotationMatrix();
        vive_time = vt->header.stamp;
        vive_init = true;
        continue;
      // Optitrack transform
      } else if (vt != NULL && vt->header.frame_id == "optitrack") {
        oPa = Eigen::Vector3d(vt->transform.translation.x,
          vt->transform.translation.y,
          vt->transform.translation.z);
        oRa = Eigen::Quaterniond(vt->transform.rotation.w,
          vt->transform.rotation.x,
          vt->transform.rotation.y,
          vt->transform.rotation.z).toRotationMatrix();
        opti_time = vt->header.stamp;
        opti_init = true;
        // Interpolation
        prev_opti_time = next_opti_time;
        next_opti_time = vt->header.stamp;
        prev_oPa = next_oPa;
        prev_oRa = next_oRa;
        next_oPa = oPa;
        next_oRa = oRa;
        prev_opti = next_opti;
        next_opti = true;
        if (!next_opti || !prev_opti ) continue;
        double prev_dt = abs((vive_time - prev_opti_time).toNSec());
        double next_dt = abs((next_opti_time - vive_time).toNSec());
        oPa = (prev_dt/(prev_dt + next_dt)) * prev_oPa +
          (next_dt/(prev_dt + next_dt)) * next_oPa;
        // std::cout << "prev_dt " << prev_dt << std::endl;
        // std::cout << "next_dt " << next_dt << std::endl;
        // std::cout << "oPa: " << oPa.transpose() << std::endl;
        //
        Eigen::Quaterniond prev_oQa(prev_oRa);
        Eigen::Quaterniond next_oQa(next_oRa);
        geometry_msgs::Quaternion prev_msg_oQa;
        prev_msg_oQa.w = prev_oQa.w();
        prev_msg_oQa.x = prev_oQa.x();
        prev_msg_oQa.y = prev_oQa.y();
        prev_msg_oQa.z = prev_oQa.z();
        geometry_msgs::Quaternion next_msg_oQa;
        next_msg_oQa.w = next_oQa.w();
        next_msg_oQa.x = next_oQa.x();
        next_msg_oQa.y = next_oQa.y();
        next_msg_oQa.z = next_oQa.z();
        ceres::Problem iproblem;
        double iA[3];
        Eigen::AngleAxisd next_auxAA(next_oRa);
        iA[0] = (next_auxAA.axis() * next_auxAA.angle())(0);
        iA[1] = (next_auxAA.axis() * next_auxAA.angle())(1);
        iA[2] = (next_auxAA.axis() * next_auxAA.angle())(2);
        ceres::CostFunction * prev_cost =
          new ceres::AutoDiffCostFunction<OrientationAverageCost, 1, 3>
          (new OrientationAverageCost(prev_msg_oQa, prev_dt));
        iproblem.AddResidualBlock(prev_cost, NULL, iA);
        ceres::CostFunction * next_cost =
          new ceres::AutoDiffCostFunction<OrientationAverageCost, 1, 3>
          (new OrientationAverageCost(next_msg_oQa, next_dt));
        iproblem.AddResidualBlock(next_cost, NULL, iA);
        ceres::Solve(options, &iproblem, &summary);
        Eigen::Vector3d iV(iA[0], iA[1], iA[2]);
        // std::cout << "next_auxAA: " << (next_auxAA.axis() * next_auxAA.angle()).transpose() << std::endl;
        // Eigen::AngleAxisd prev_auxAA(prev_oRa);
        // std::cout << "prev_auxAA: " << (prev_auxAA.axis() * prev_auxAA.angle()).transpose() << std::endl;
        // std::cout << "iV: " << iV.transpose() << std::endl;
        Eigen::AngleAxisd iAA(iV.norm(), iV.normalized());
        oRa = iAA.toRotationMatrix();
      }
    }
    if (!vive_init || !opti_init || !next_opti || !prev_opti) continue;
    // Transform
    if (((opti_time - vive_time).toSec()) < 0.05) {
      Eigen::Matrix3d est_aRt = oRa.transpose() * oRv * vRt;
      Eigen::Vector3d est_aPt = oRa.transpose() * (
        oRv * vPt + oPv) + (-oRa.transpose() * oPa);
      Eigen::AngleAxisd dAA(est_aRt.transpose() * aRt);
      Eigen::Matrix3d dR = est_aRt.transpose() * aRt;
      Eigen::AngleAxisd oAa(oRa);
      Eigen::Vector3d oVa = oAa.axis() * oAa.angle();
      Eigen::AngleAxisd vAt(vRt);
      Eigen::Vector3d vVt = vAt.axis() * vAt.angle();
      // Position difference
      Eigen::Vector3d dP = est_aPt - aPt;
      sP += dP;
      sD += dP.norm();
      // std::cout << dP.norm() << " ";
      if (dP.norm() > mD) mD = dP.norm();
      // Orientation averaging
      Eigen::Quaterniond dQ(dR);
      geometry_msgs::Quaternion msgQ;
      msgQ.w = dQ.w();
      msgQ.x = dQ.x();
      msgQ.y = dQ.y();
      msgQ.z = dQ.z();
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<OrientationAverageCost, 1, 3>
        (new OrientationAverageCost(msgQ, 1.0));
      problem.AddResidualBlock(cost, NULL, dA);
      // std::cout << dAA.angle() << " ";
      sA += dAA.angle();
      if (dAA.angle() > mA) mA = dAA.angle();
      // Prints
      // std::cout << (sP / pose_counter).transpose() << std::endl;
      pose_counter++;
      // std::cout << " - " << dP.norm() << std::endl;
     }
  }

  ceres::Solve(options, &problem, &summary);

  std::cout << "\nCalibration Quality:\n";
  std::cout << "Average Position Offset: " << (sP / pose_counter).transpose() << std::endl;
  std::cout << "Average Distance: " << (sP / pose_counter).norm() << std::endl;
  std::cout << "Average Rotation Offset: " << 
    dA[0] << ", " << dA[1] << ", " << dA[2] << std::endl;
  std::cout << "Average Angle: " << (180.0 / M_PI) * sqrt(dA[0]*dA[0] +
    dA[1]*dA[1] +
    dA[2]*dA[2]) << std::endl;



  std::cout << "\nTracking Quality:\n";
  std::cout << "Average Distance: " << sD / pose_counter << std::endl;
  std::cout << "Max Distance: " << mD << std::endl;
  std::cout << "Average Angle: " << (180.0 / M_PI) * sA / pose_counter << std::endl;
  std::cout << "Max Angle: " << (180.0 / M_PI) * mA << std::endl;

  data_bag.close();

  return 0;
}