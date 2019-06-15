// This package code
#include <hive/vive.h>
#include <hive/vive_general.h>

// Standard C includes
#include <stdio.h>
#include <stdlib.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Standard C++ includes
#include <iostream>
#include <string>
#include <random>

// Services
#include <hive/ViveConfig.h>

// Messages
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
  ceres::AngleAxisToRotationMatrix(average, Raverage.data());

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

  rosbag::View view;
  rosbag::Bag data_bag, offset_bag;

  std::string offset_filename(argv[1]);
  std::string data_filename(argv[2]);

  offset_bag.open(argv[1], rosbag::bagmode::Read);
  data_bag.open(argv[2], rosbag::bagmode::Read);

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

  // Saving poses
  geometry_msgs::TransformStamped opti_msg, vive_msg;
  ros::Time opti_time(0), vive_time(0);
  Eigen::Matrix3d vRt, oRa;
  Eigen::Vector3d vPt, oPa;
  // Interpolation
  Eigen::Matrix3d prev_oRa, next_oRa;
  Eigen::Vector3d prev_oPa, next_oPa;
  ros::Time prev_opti_time, next_opti_time;

  // Position data
  std::vector<double> vive_px;
  std::vector<double> vive_py;
  std::vector<double> vive_pz;

  std::vector<double> vive_qw;
  std::vector<double> vive_qx;
  std::vector<double> vive_qy;
  std::vector<double> vive_qz;

  std::vector<double> vive_t;

  std::vector<double> opti_px;
  std::vector<double> opti_py;
  std::vector<double> opti_pz;

  std::vector<double> opti_qw;
  std::vector<double> opti_qx;
  std::vector<double> opti_qy;
  std::vector<double> opti_qz;

  std::vector<double> opti_t;

  std::vector<double> dist_v;
  std::vector<double> ang_v;
  std::vector<double> time_v;

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
  // Cut the bag
  double time0 = -1;
  // Scan bag
  rosbag::View view_li(data_bag, rosbag::TopicQuery(topics));

  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
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

        // Save pose
        Eigen::Vector3d est_oPa = oRv * ( vRt * ( - aRt.transpose() * aPt) + vPt) + oPv;
        Eigen::Matrix3d est_oRa = oRv * vRt * aRt.transpose();
        if (time0 != -1) {
          vive_px.push_back(est_oPa(0));
          vive_py.push_back(est_oPa(1));
          vive_pz.push_back(est_oPa(2));
          Eigen::Quaterniond est_oQa(est_oRa);
          vive_qw.push_back(est_oQa.w());
          vive_qx.push_back(est_oQa.x());
          vive_qy.push_back(est_oQa.y());
          vive_qz.push_back(est_oQa.z());
          vive_t.push_back(vive_time.toSec() - time0);
        }

        continue;
      // Optitrack transform
      } else if (vt != NULL && vt->header.frame_id == "optitrack") {
        if (time0 == -1) time0 = vt->header.stamp.toSec();

        oPa = Eigen::Vector3d(vt->transform.translation.x,
          vt->transform.translation.y,
          vt->transform.translation.z);
        oRa = Eigen::Quaterniond(vt->transform.rotation.w,
          vt->transform.rotation.x,
          vt->transform.rotation.y,
          vt->transform.rotation.z).toRotationMatrix();
        opti_time = vt->header.stamp;
        opti_init = true;

        opti_px.push_back(oPa(0));
        opti_py.push_back(oPa(1));
        opti_pz.push_back(oPa(2));
        Eigen::Quaterniond oQa(oRa);
        opti_qw.push_back(oQa.w());
        opti_qx.push_back(oQa.x());
        opti_qy.push_back(oQa.y());
        opti_qz.push_back(oQa.z());
        opti_t.push_back(opti_time.toSec() - time0);

        // vive_init = false;
        // Interpolation
        prev_opti_time = next_opti_time;
        next_opti_time = vt->header.stamp;
        if (prev_opti_time > vive_time) continue;

        prev_oPa = next_oPa;
        prev_oRa = next_oRa;
        next_oPa = Eigen::Vector3d(vt->transform.translation.x,
          vt->transform.translation.y,
          vt->transform.translation.z);
        next_oRa = Eigen::Quaterniond(vt->transform.rotation.w,
          vt->transform.rotation.x,
          vt->transform.rotation.y,
          vt->transform.rotation.z).toRotationMatrix();
        prev_opti = next_opti;
        next_opti = true;

        if (true) {
          // // Data10.9
          if (data_filename.find("data10.9") != std::string::npos) {
            if ((prev_opti_time.toSec() - time0 > 8.803)
              && (prev_opti_time.toSec() - time0 < 9.789) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 10.58)
              && (prev_opti_time.toSec() - time0 < 11.21) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 14.75)
              && (prev_opti_time.toSec() - time0 < 15.67) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 16.72)
              && (prev_opti_time.toSec() - time0 < 17.95) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 21.96)
              && (prev_opti_time.toSec() - time0 < 22.53) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 22.99)
              && (prev_opti_time.toSec() - time0 < 23.25) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 7.345)
              && (prev_opti_time.toSec() - time0 < 7.855) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 18.35)
              && (prev_opti_time.toSec() - time0 < 19.91) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 8.803)
              && (next_opti_time.toSec() - time0 < 9.789) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 10.58)
              && (next_opti_time.toSec() - time0 < 11.21) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 14.75)
              && (next_opti_time.toSec() - time0 < 15.67) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 16.72)
              && (next_opti_time.toSec() - time0 < 17.95) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 21.96)
              && (next_opti_time.toSec() - time0 < 22.53) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 22.99)
              && (next_opti_time.toSec() - time0 < 23.25) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 7.345)
              && (next_opti_time.toSec() - time0 < 7.855) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 18.35)
              && (next_opti_time.toSec() - time0 < 19.91) ) {
              continue;
            }
          }






          // // Data10.10
          if (data_filename.find("data10.10") != std::string::npos) {
            if ((prev_opti_time.toSec() - time0 > 6.797)
              && (prev_opti_time.toSec() - time0 < 8.317) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 11.7)
              && (prev_opti_time.toSec() - time0 < 11.98) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 12.85)
              && (prev_opti_time.toSec() - time0 < 13.1) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 14.4)
              && (prev_opti_time.toSec() - time0 < 14.65) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 19.5)
              && (prev_opti_time.toSec() - time0 < 20.2) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 23.31)
              && (prev_opti_time.toSec() - time0 < 23.54) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 25.46)
              && (prev_opti_time.toSec() - time0 < 25.75) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 29.06)
              && (prev_opti_time.toSec() - time0 < 29.65) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 6.797)
              && (next_opti_time.toSec() - time0 < 8.317) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 11.7)
              && (next_opti_time.toSec() - time0 < 11.98) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 12.85)
              && (next_opti_time.toSec() - time0 < 13.1) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 14.4)
              && (next_opti_time.toSec() - time0 < 14.65) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 19.5)
              && (next_opti_time.toSec() - time0 < 20.2) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 23.31)
              && (next_opti_time.toSec() - time0 < 23.54) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 25.46)
              && (next_opti_time.toSec() - time0 < 25.75) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 29.06)
              && (next_opti_time.toSec() - time0 < 29.65) ) {
              continue;
            }
          }





          // // Data10.11
          if (data_filename.find("data10.11") != std::string::npos) {
            if ((prev_opti_time.toSec() - time0 > 1.068)
              && (prev_opti_time.toSec() - time0 < 1.586) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 5.33)
              && (prev_opti_time.toSec() - time0 < 5.546) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 9.096)
              && (prev_opti_time.toSec() - time0 < 9.676) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 11.73)
              && (prev_opti_time.toSec() - time0 < 12.2) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 13.27)
              && (prev_opti_time.toSec() - time0 < 13.64) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 16.06)
              && (prev_opti_time.toSec() - time0 < 16.46) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 17.7)
              && (prev_opti_time.toSec() - time0 < 18.08) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 1.068)
              && (next_opti_time.toSec() - time0 < 1.586) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 5.33)
              && (next_opti_time.toSec() - time0 < 5.546) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 9.096)
              && (next_opti_time.toSec() - time0 < 9.676) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 11.73)
              && (next_opti_time.toSec() - time0 < 12.2) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 13.27)
              && (next_opti_time.toSec() - time0 < 13.64) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 16.06)
              && (next_opti_time.toSec() - time0 < 16.46) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 17.7)
              && (next_opti_time.toSec() - time0 < 18.08) ) {
              continue;
            }
          }





          
          // // Data10.12
          if (data_filename.find("data10.12") != std::string::npos) {
            if ((prev_opti_time.toSec() - time0 > 7.413)
              && (prev_opti_time.toSec() - time0 < 11.21) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 11.63)
              && (prev_opti_time.toSec() - time0 < 12.25) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 16.2)
              && (prev_opti_time.toSec() - time0 < 16.31) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 19.68)
              && (prev_opti_time.toSec() - time0 < 20.28) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 13.2)
              && (prev_opti_time.toSec() - time0 < 13.45) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 13.84)
              && (prev_opti_time.toSec() - time0 < 14.06) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 7.413)
              && (next_opti_time.toSec() - time0 < 11.21) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 11.63)
              && (next_opti_time.toSec() - time0 < 12.25) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 16.2)
              && (next_opti_time.toSec() - time0 < 16.31) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 19.68)
              && (next_opti_time.toSec() - time0 < 20.28) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 13.2)
              && (next_opti_time.toSec() - time0 < 13.45) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 13.84)
              && (next_opti_time.toSec() - time0 < 14.06) ) {
              continue;
            }
          }




          // Data10.13
          if (data_filename.find("data10.13") != std::string::npos) {
            if ((prev_opti_time.toSec() - time0 > 10.22)
              && (prev_opti_time.toSec() - time0 < 10.82) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 18.91)
              && (prev_opti_time.toSec() - time0 < 19.62) ) {
              continue;
            }

            if ((prev_opti_time.toSec() - time0 > 23.8)
              && (prev_opti_time.toSec() - time0 < 24.12) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 10.22)
              && (next_opti_time.toSec() - time0 < 10.82) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 18.91)
              && (next_opti_time.toSec() - time0 < 19.62) ) {
              continue;
            }

            if ((next_opti_time.toSec() - time0 > 23.8)
              && (next_opti_time.toSec() - time0 < 24.12) ) {
              continue;
            }
          }
        }

        // std::cout << "dT: " << (next_opti_time - prev_opti_time).toSec() << std::endl;
        if ((next_opti_time - prev_opti_time).toSec() > 0.025) continue;

        if (!next_opti || !prev_opti ) continue;
        double prev_dt = abs((vive_time - prev_opti_time).toNSec());
        double next_dt = abs((next_opti_time - vive_time).toNSec());
        oPa = (prev_dt/(prev_dt + next_dt)) * prev_oPa +
          (next_dt/(prev_dt + next_dt)) * next_oPa;

        // std::cout << next_opti_time.toSec() - time0 << " ";
        // std::cout << oPa(0) << " ";

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
        // Eigen::Quaterniond auxQ(oRa);
        // std::cout << next_opti_time.toSec() - time0 << " ";
        // std::cout << auxQ.w() << " ";

        // Save pose
        // if (vive_init) {
        //   // Eigen::Vector3d tmp_vPt = oRv.transpose() * (oRa * aPt + oPa) -
        //   //   oRv.transpose() * oPv;
        //   // Eigen::Matrix3d tmp_vRt = oRv.transpose() * oRa * aRt;
        //   opti_px.push_back(oPa(0));
        //   opti_py.push_back(oPa(1));
        //   opti_pz.push_back(oPa(2));
        //   Eigen::Quaterniond oQa(oRa);
        //   opti_qw.push_back(oQa.w());
        //   opti_qx.push_back(oQa.x());
        //   opti_qy.push_back(oQa.y());
        //   opti_qz.push_back(oQa.z());
        //   // std::cout << vive_time.toSec() << std::endl;
        //   // std::cout << time0 << std::endl;
        //   // std::cout << vive_time.toSec() - time0 << std::endl;
        //   opti_t.push_back(vive_time.toSec() - time0);
        // }
      }
    }
    if (!vive_init || !opti_init || !next_opti || !prev_opti) continue;
    // Transform
    if (((opti_time - vive_time).toSec()) < 0.05) {
      // Calibration
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

      // Tracking
      // Eigen::Vector3d est_oPa = oRv * ( vRt * ( - aRt.transpose() * aPt) + vPt) + oPv;
      // Eigen::Matrix3d est_oRa = oRv * vRt * aRt.transpose();
      Eigen::Vector3d est_vPt = oRv.transpose() * (oRa * aPt + oPa) -
        oRv.transpose() * oPv;
      Eigen::Matrix3d est_vRt = oRv.transpose() * oRa * aRt;
      // Eigen::Vector3d d_oPa = est_oPa - oPa;
      // Eigen::AngleAxisd d_oAa(est_oRa.transpose() * oRa);
      Eigen::Vector3d d_vPt = est_vPt - vPt;
      Eigen::AngleAxisd d_vAt(est_vRt.transpose() * vRt);
      // Average Error
      sD += d_vPt.norm();
      dist_v.push_back(d_vPt.norm());
      // Maximum Error
      if (d_vPt.norm() > mD) mD = d_vPt.norm();
      // Average Error
      sA += d_vAt.angle();
      ang_v.push_back(d_vAt.angle());
      // Maximum Error
      if (d_vAt.angle() > mA) mA = d_vAt.angle();

      time_v.push_back(vive_time.toSec() - time0);


      pose_counter++;
     }
  }

  ceres::Solve(options, &problem, &summary);

  data_bag.close();

  std::cout << "vive_px = [";
  for (auto pos : vive_px) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_py = [";
  for (auto pos : vive_py) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_pz = [";
  for (auto pos : vive_pz) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_qw = [";
  for (auto pos : vive_qw) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_qx = [";
  for (auto pos : vive_qx) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_qy = [";
  for (auto pos : vive_qy) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_qz = [";
  for (auto pos : vive_qz) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "vive_t = [";
  for (auto pos : vive_t) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;




  std::cout << "opti_px = [";
  for (auto pos : opti_px) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_py = [";
  for (auto pos : opti_py) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_pz = [";
  for (auto pos : opti_pz) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_qw = [";
  for (auto pos : opti_qw) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_qx = [";
  for (auto pos : opti_qx) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_qy = [";
  for (auto pos : opti_qy) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_qz = [";
  for (auto pos : opti_qz) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "opti_t = [";
  for (auto pos : opti_t) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << std::endl;

  std::cout << "dist_v = [";
  for (auto pos : dist_v) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "ang_v = [";
  for (auto pos : ang_v) {
    std::cout << 180 / M_PI * pos << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "time_v = [";
  for (auto pos : time_v) {
    std::cout << pos << " ";
  }
  std::cout << "]" << std::endl;

  return 0;
}