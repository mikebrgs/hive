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

bool isOutlier(std::string filename, double time) {
  // // Data10.9
  if (filename.find("data10.9") != std::string::npos) {
    if ((time > 8.803)
      && (time < 9.789) ) {
      return true;
    }

    if ((time > 10.58)
      && (time < 11.21) ) {
      return true;
    }

    if ((time > 14.75)
      && (time < 15.67) ) {
      return true;
    }

    if ((time > 16.72)
      && (time < 17.95) ) {
      return true;
    }

    if ((time > 21.96)
      && (time < 22.53) ) {
      return true;
    }

    if ((time > 22.99)
      && (time < 23.25) ) {
      return true;
    }

    if ((time > 7.345)
      && (time < 7.855) ) {
      return true;
    }

    if ((time > 18.35)
      && (time < 19.91) ) {
      return true;
    }
  }


  // // Data10.10
  if (filename.find("data10.10") != std::string::npos) {
    if ((time > 6.797)
      && (time < 8.317) ) {
      return true;
    }

    if ((time > 11.7)
      && (time < 11.98) ) {
      return true;
    }

    if ((time > 12.85)
      && (time < 13.1) ) {
      return true;
    }

    if ((time > 14.4)
      && (time < 14.65) ) {
      return true;
    }

    if ((time > 19.5)
      && (time < 20.2) ) {
      return true;
    }

    if ((time > 23.31)
      && (time < 23.54) ) {
      return true;
    }

    if ((time > 25.46)
      && (time < 25.75) ) {
      return true;
    }

    if ((time > 29.06)
      && (time < 29.65) ) {
      return true;
    }
  }


  // // Data10.11
  if (filename.find("data10.11") != std::string::npos) {
    if ((time > 1.068)
      && (time < 1.586) ) {
      return true;
    }

    if ((time > 5.33)
      && (time < 5.546) ) {
      return true;
    }

    if ((time > 9.096)
      && (time < 9.676) ) {
      return true;
    }

    if ((time > 11.73)
      && (time < 12.2) ) {
      return true;
    }

    if ((time > 13.27)
      && (time < 13.64) ) {
      return true;
    }

    if ((time > 16.06)
      && (time < 16.46) ) {
      return true;
    }

    if ((time > 17.7)
      && (time < 18.08) ) {
      return true;
    }
  }


  // // Data10.12
  if (filename.find("data10.12") != std::string::npos) {
    if ((time > 7.413)
      && (time < 11.21) ) {
      return true;
    }

    if ((time > 11.63)
      && (time < 12.25) ) {
      return true;
    }

    if ((time > 16.2)
      && (time < 16.31) ) {
      return true;
    }

    if ((time > 19.68)
      && (time < 20.28) ) {
      return true;
    }

    if ((time > 13.2)
      && (time < 13.45) ) {
      return true;
    }

    if ((time > 13.84)
      && (time < 14.06) ) {
      return true;
    }
  }


  // Data10.13
  if (filename.find("data10.13") != std::string::npos) {
    if ((time > 10.22)
      && (time < 10.82) ) {
      return true;
    }

    if ((time > 18.91)
      && (time < 19.62) ) {
      return true;
    }

    if ((time > 23.8)
      && (time < 24.12) ) {
      return true;
    }
  }
  return false;
}

int main(int argc, char ** argv) {

  if (argc < 3) {
    std::cout << "rosrun hive hive_print_offset"
      << "<offset_cal>.bag <data>.bag" << std::endl;
      return -1;
  }


  // Read Offset poses
  std::string offset_filename(argv[1]);
  rosbag::Bag offset_bag;
  offset_bag.open(argv[1], rosbag::bagmode::Read);
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

  // Position
  Eigen::Vector3d sP = Eigen::Vector3d::Zero();
  double sD = 0.0;
  double mD = 0.0;
  ros::Time mDtime;
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
  ros::Time mAtime;
  // Counter
  double pose_counter = 0;
  // Initializations
  bool vive_init = false, opti_init = false;
  bool prev_opti = false, next_opti = false;
  // Cut the bag
  double time0 = -1;

  // Scan bag
  rosbag::Bag data_bag;
  std::string data_filename(argv[2]);
  data_bag.open(argv[2], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("tf");
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
        // Eigen::Vector3d est_oPa = oRv * ( vRt * ( - aRt.transpose() * aPt) + vPt) + oPv;
        // Eigen::Matrix3d est_oRa = oRv * vRt * aRt.transpose();

        // std::cout << "VIVE " << vive_time << " - "
        //   << vPt(0) << ", "
        //   << vPt(1) << ", "
        //   << vPt(2) << std::endl;

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

        Eigen::Vector3d est_vPt = oRv.transpose() * (oRa * aPt + oPa) -
          oRv.transpose() * oPv;
        Eigen::Matrix3d est_vRt = oRv.transpose() * oRa * aRt;

        // std::cout << "OPTI " << prev_opti_time << " - "
        //   << est_vPt(0) << ", "
        //   << est_vPt(1) << ", "
        //   << est_vPt(2) << std::endl;

        // Interpolation
        prev_opti_time = next_opti_time;
        next_opti_time = vt->header.stamp;

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

        if (prev_opti_time > vive_time) continue;

        if (true) {
          if (isOutlier(data_filename, prev_opti_time.toSec() - time0)) {
            continue;
          }
          if (isOutlier(data_filename, next_opti_time.toSec() - time0)) {
            continue;
          }
        }

        if ((next_opti_time - prev_opti_time).toSec() > 0.025) continue;

        if (!next_opti || !prev_opti ) continue;
        double prev_dt = abs((vive_time - prev_opti_time).toNSec());
        double next_dt = abs((next_opti_time - vive_time).toNSec());
        oPa = (prev_dt/(prev_dt + next_dt)) * prev_oPa +
          (next_dt/(prev_dt + next_dt)) * next_oPa;


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

        Eigen::AngleAxisd iAA(iV.norm(), iV.normalized());
        oRa = iAA.toRotationMatrix();


        Eigen::Vector3d tmp_vPt = oRv.transpose() * (oRa * aPt + oPa) -
          oRv.transpose() * oPv;
        Eigen::Matrix3d tmp_vRt = oRv.transpose() * oRa * aRt;

        // std::cout << "PRED OPTI " << vive_time << " - "
        //   << tmp_vPt(0) << ", "
        //   << tmp_vPt(1) << ", "
        //   << tmp_vPt(2) << std::endl;
      }
    }
    if (!vive_init || !opti_init || !next_opti || !prev_opti) continue;
    // Transform
    if (((opti_time - vive_time).toSec()) < 0.02) {
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
      Eigen::Vector3d est_vPt = oRv.transpose() * (oRa * aPt + oPa) -
        oRv.transpose() * oPv;
      Eigen::Matrix3d est_vRt = oRv.transpose() * oRa * aRt;
      Eigen::Vector3d d_vPt = est_vPt - vPt;
      Eigen::AngleAxisd d_vAt(est_vRt.transpose() * vRt);
      // Average Error
      sD += d_vPt.norm();
      // Maximum Error
      if (d_vPt.norm() > mD) {
        mD = d_vPt.norm();
        mDtime = vive_time;
      }
      // Average Error
      sA += d_vAt.angle();
      // Maximum Error
      if (d_vAt.angle() > mA) {
        mA = d_vAt.angle();
        mAtime = vive_time;
      }

      pose_counter++;
     }
  }

  ceres::Solve(options, &problem, &summary);

  data_bag.close();

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
  std::cout << "Distance Time: " << mDtime << std::endl;
  std::cout << "Average Angle: " << (180.0 / M_PI) * sA / pose_counter << std::endl;
  std::cout << "Max Angle: " << (180.0 / M_PI) * mA << std::endl;
  std::cout << "Angle Time: " << mAtime << std::endl;

  return 0;
}

