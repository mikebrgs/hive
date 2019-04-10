/*
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
#include <math.h>

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

// Visp
#include <visp3/vision/vpHandEyeCalibration.h>

// ROS msgs
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

#define INFUNCTIONRESIDUALS 4
#define DISTANCE_THRESH 0.1
#define ANGLE_THRESH 0.17
#define TIME_THRESH 0.01

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;

struct PoseCostFunctor{
  explicit PoseCostFunctor(TF vive, TF optitrack) :
    vive_(vive), optitrack_(optitrack) {}
  template <typename T> bool operator()(const T* const o_v,
    const T* const a_t,
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

    vPt << T(vive_.transform.translation.x),
        T(vive_.transform.translation.y),
        T(vive_.transform.translation.z);
    vRt << Eigen::Quaternion<T>(T(vive_.transform.rotation.w),
        T(vive_.transform.rotation.x),
        T(vive_.transform.rotation.y),
        T(vive_.transform.rotation.z)).normalized().toRotationMatrix();
    oPa << T(optitrack_.transform.translation.x),
        T(optitrack_.transform.translation.y),
        T(optitrack_.transform.translation.z);
    oRa << Eigen::Quaternion<T>(T(optitrack_.transform.rotation.w),
        T(optitrack_.transform.rotation.x),
        T(optitrack_.transform.rotation.y),
        T(optitrack_.transform.rotation.z)).normalized().toRotationMatrix();
    oPv << o_v[0],
        o_v[1],
        o_v[2];
    ceres::AngleAxisToRotationMatrix(&o_v[3], oRv.data());
    aPt << a_t[0],
        a_t[1],
        a_t[2];
    ceres::AngleAxisToRotationMatrix(&a_t[3], aRt.data());

    // Convert Vive to OptiTrack
    Eigen::Matrix<T, 3, 1> tPa = -aRt.transpose() * aPt;
    Eigen::Matrix<T, 3, 3> tRa = aRt.transpose();

    Eigen::Matrix<T, 3, 1> _vPa = vRt * tPa + vPt;
    Eigen::Matrix<T, 3, 3> _vRa = vRt * tRa;

    Eigen::Matrix<T, 3, 1> _oPa = oRv * _vPa + oPv;
    Eigen::Matrix<T, 3, 3> _oRa = oRv * _vRa;

    Eigen::Quaternion<T> oQa(oRa);
    oQa.normalize();
    Eigen::Quaternion<T> _oQa(_oRa);
    _oQa.normalize();

    T delta = T(vive_.header.stamp.toSec()) - T(optitrack_.header.stamp.toSec());
    delta = T(1.0);
    residual[0] = (_oPa(0) - oPa(0)) / (T(0.01) + delta);
    residual[1] = (_oPa(1) - oPa(1)) / (T(0.01) + delta);
    residual[2] = (_oPa(2) - oPa(2)) / (T(0.01) + delta);

    T aa[3];
    Eigen::Matrix<T, 3, 3> R = _oRa * oRa.transpose();
    ceres::RotationMatrixToAngleAxis(R.data(), aa);
    residual[3] = sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);

    return true;
  }
 private:
  TF vive_;
  TF optitrack_;
};

TFs RefineOffset(TFs optitrack, TFs vive, TFs offset) {
  // Format conversion for ceres
  // TF aTt = offset[0]; // Transform from tracker to optitrack's arrow
  vpTranslationVector aPt(offset[0].transform.translation.x,
    offset[0].transform.translation.y,
    offset[0].transform.translation.z);
  vpQuaternionVector aQt(offset[0].transform.rotation.x,
    offset[0].transform.rotation.y,
    offset[0].transform.rotation.z,
    offset[0].transform.rotation.w);
  vpThetaUVector aAt(aQt);
  // TF oTv = offset[1]; // Transform from vive to optitrack
  vpTranslationVector oPv(offset[1].transform.translation.x,
    offset[1].transform.translation.y,
    offset[1].transform.translation.z);
  vpQuaternionVector oQv(offset[1].transform.rotation.x,
    offset[1].transform.rotation.y,
    offset[1].transform.rotation.z,
    offset[1].transform.rotation.w);
  vpThetaUVector oAv(oQv);

  double aTt[6] = {
    aPt[0], aPt[1], aPt[2],
    aAt[0], aAt[1], aAt[2]
  };
  double oTv[6] = {
    oPv[0], oPv[1], oPv[2],
    oAv[0], oAv[1], oAv[2]
  };

  // Ceres problem
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Solver's options
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;

  // Different notation
  std::cout << "oTv: "
            << oTv[0] << ", "
            << oTv[1] << ", "
            << oTv[2] << ", "
            << oTv[3] << ", "
            << oTv[4] << ", "
            << oTv[5] << std::endl;
  std::cout << "aTt: "
            << aTt[0] << ", "
            << aTt[1] << ", "
            << aTt[2] << ", "
            << aTt[3] << ", "
            << aTt[4] << ", "
            << aTt[5] << std::endl;

  auto v_it = vive.begin();
  auto o_it = optitrack.begin();
  while (v_it != vive.end() && o_it != optitrack.end()) {
    ceres::CostFunction * thecost =
      new ceres::AutoDiffCostFunction<PoseCostFunctor, INFUNCTIONRESIDUALS, 6, 6>
      (new PoseCostFunctor(*v_it, *o_it));
    problem.AddResidualBlock(thecost, NULL, oTv, aTt);
    v_it++;
    o_it++;
  }

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
  std::cout << "oTv: "
            << oTv[0] << ", "
            << oTv[1] << ", "
            << oTv[2] << ", "
            << oTv[3] << ", "
            << oTv[4] << ", "
            << oTv[5] << std::endl;
  std::cout << "aTt: "
            << aTt[0] << ", "
            << aTt[1] << ", "
            << aTt[2] << ", "
            << aTt[3] << ", "
            << aTt[4] << ", "
            << aTt[5] << std::endl;

  // aTt - Transform from tracker to optitrack's arrow
  aQt = vpQuaternionVector(
    vpThetaUVector(aTt[3], aTt[4], aTt[5]));
  offset[0].transform.translation.x = aTt[0];
  offset[0].transform.translation.y = aTt[1];
  offset[0].transform.translation.z = aTt[2];
  offset[0].transform.rotation.w = aQt.w();
  offset[0].transform.rotation.x = aQt.x();
  offset[0].transform.rotation.y = aQt.y();
  offset[0].transform.rotation.z = aQt.z();

  // oTv - Transform from vive to optitrack
  oQv = vpQuaternionVector(
    vpThetaUVector(oTv[3], oTv[4], oTv[5]));
  offset[1].transform.translation.x = oTv[0];
  offset[1].transform.translation.y = oTv[1];
  offset[1].transform.translation.z = oTv[2];
  offset[1].transform.rotation.w = oQv.w();
  offset[1].transform.rotation.x = oQv.x();
  offset[1].transform.rotation.y = oQv.y();
  offset[1].transform.rotation.z = oQv.z();

  return offset;
}

TFs EstimateOffset(TFs optitrack, TFs vive) {
  std::vector<vpHomogeneousMatrix> vMtVec; // camera to object - tracker to vive
  std::vector<vpHomogeneousMatrix> tMvVec;
  std::vector<vpHomogeneousMatrix> aMoVec; // reference to end effector - optitrack to arrow
  std::vector<vpHomogeneousMatrix> oMaVec;
  // vpHomogeneousMatrix tMa; // end effector to camera - arrow to tracker
  // vpHomogeneousMatrix vMo; // TODO Check this one

  // Tracker in Vive's frame
  for (auto msg_it = vive.begin();
    msg_it != vive.end(); msg_it++) {
    vpHomogeneousMatrix vMt;
    vpTranslationVector vtt(msg_it->transform.translation.x,
      msg_it->transform.translation.y,
      msg_it->transform.translation.z);
    vpQuaternionVector vrt(msg_it->transform.rotation.x,
      msg_it->transform.rotation.y,
      msg_it->transform.rotation.z,
      msg_it->transform.rotation.w);
    vMt.buildFrom(vtt, vrt);
    vMtVec.push_back(vMt); // To compute the big offset
    tMvVec.push_back(vMt.inverse()); // To compute the small offset
}

  // Arrow in Optitrack's frame
  for (auto msg_it = optitrack.begin();
    msg_it != optitrack.end(); msg_it++) {
    vpHomogeneousMatrix oMa;
    vpTranslationVector ota(msg_it->transform.translation.x,
      msg_it->transform.translation.y,
      msg_it->transform.translation.z);
    vpQuaternionVector ora(msg_it->transform.rotation.x,
      msg_it->transform.rotation.y,
      msg_it->transform.rotation.z,
      msg_it->transform.rotation.w);
    oMa.buildFrom(ota, ora);
    aMoVec.push_back(oMa.inverse()); // To compute the big offset
    oMaVec.push_back(oMa); // To compute the small offset
}

  int status;

  vpHomogeneousMatrix aMt;
  status = vpHandEyeCalibration::calibrate(tMvVec, oMaVec, aMt); // To compute the small offset
  std::cout << "aMt: ";
  aMt.print(); // end effector to camera - arrow to tracker
  std::cout << std::endl << "Status: " <<  status << std::endl;

  vpHomogeneousMatrix oMv;
  status = vpHandEyeCalibration::calibrate(vMtVec, aMoVec, oMv); // To compute the small offset
  std::cout << "oMv: ";
  oMv.print(); // end effector to camera - arrow to tracker
  std::cout << std::endl << "Status: " <<  status << std::endl;

  TFs Ts;
  TF aTt; // Transform from tracker to optitrack's arrow
  TF oTv; // Transform from vive to optitrack

  // Changing the format for aTt
  vpTranslationVector aPt;
  vpQuaternionVector aQt;
  aMt.extract(aPt);
  aMt.extract(aQt);
  aTt.transform.translation.x = aPt[0];
  aTt.transform.translation.y = aPt[1];
  aTt.transform.translation.z = aPt[2];
  aTt.transform.rotation.w = aQt.w();
  aTt.transform.rotation.x = aQt.x();
  aTt.transform.rotation.y = aQt.y();
  aTt.transform.rotation.z = aQt.z();

  // Changing the format for oTv
  vpTranslationVector oPv;
  vpQuaternionVector oQv;
  oMv.extract(oPv);
  oMv.extract(oQv);
  oTv.transform.translation.x = oPv[0];
  oTv.transform.translation.y = oPv[1];
  oTv.transform.translation.z = oPv[2];
  oTv.transform.rotation.w = oQv.w();
  oTv.transform.rotation.x = oQv.x();
  oTv.transform.rotation.y = oQv.y();
  oTv.transform.rotation.z = oQv.z();

  Ts.push_back(aTt);
  Ts.push_back(oTv);
  return Ts;
}

void TestTrajectory(TFs & vive, TFs & optitrack) {
  vpTranslationVector oPv(-2,-1,1.7);
  vpThetaUVector oAv(0,0,0);
  vpHomogeneousMatrix oMv(oPv, oAv);
  vpTranslationVector aPt(0.01,0.06,0.02);
  vpThetaUVector aAt(0,0,0);
  vpHomogeneousMatrix aMt(aPt, aAt);

  for (double i = 0; i < 100; i++) {
    vpTranslationVector vPt(cos(M_PI / 13.576 * i),sin(M_PI / 13.576 *i),i);
    vpThetaUVector vAt(sqrt(i),i,i*i);
    vpHomogeneousMatrix vMt(vPt, vAt);
    vpHomogeneousMatrix oMa = oMv * vMt * aMt.inverse();
    vpTranslationVector oPa;
    vpQuaternionVector oQa, vQt;
    vMt.extract(vQt);
    oMa.extract(oQa);
    oMa.extract(oPa);
    TF oTa, vTt;
    oTa.transform.translation.x = oPa[0] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.translation.y = oPa[1] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.translation.z = oPa[2] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.rotation.w = oQa.w() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.rotation.x = oQa.x() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.rotation.y = oQa.y() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    oTa.transform.rotation.z = oQa.z() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.translation.x = vPt[0] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.translation.y = vPt[1] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.translation.z = vPt[2] + 0.005 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.rotation.w = vQt.w() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.rotation.x = vQt.x() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.rotation.y = vQt.y() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vTt.transform.rotation.z = vQt.z() + 0.001 * ((double)rand() / RAND_MAX * 2.0 - 1.0);
    vive.push_back(vTt);
    optitrack.push_back(oTa);
  }
  return;
}

class HiveOffset
{
public:
  HiveOffset(int argc, char ** argv);
  ~HiveOffset();
  void Spin();
private:
  std::string bagname_;
  std::vector<geometry_msgs::TransformStamped> ov_;
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
      ov_.push_back(*tf_it);
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
  // return;


  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal_.SetTrackers(*vt);
  }
  ROS_INFO("Trackers' setup complete.");

  // Temporary solver - Easily changed (one tracker only)
  solver = new BaseSolve(cal_.environment,
    cal_.trackers.begin()->second);

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    solver->ProcessLight(vl);
    geometry_msgs::TransformStamped msg;
    if (solver->GetTransform(msg)) {
      vv_.push_back(msg);
    }
  }
  ROS_INFO("Data processment complete.");

  TFs vive_v, opti_v;
  TFs vive_vfull, opti_vfull;
  // Search the vive poses
  for (auto vive_it = vv_.begin(); vive_it != vv_.end(); vive_it++) {
    // Search the optitrack poses
    for (auto opti_it = ov_.begin(); opti_it != ov_.end(); opti_it++) {
      // Check if the optitrack iterator is after the vive pose
      if (opti_it->header.stamp.toSec() > vive_it->header.stamp.toSec()) {
        // Find the closes image
        if (opti_it == ov_.begin()) {
          vive_vfull.push_back(*vive_it);
          opti_vfull.push_back(*opti_it);
        } else if (opti_it->header.stamp.toSec() - vive_it->header.stamp.toSec() <
          - (opti_it - 1)->header.stamp.toSec() + vive_it->header.stamp.toSec()) {
          vive_vfull.push_back(*vive_it);
          opti_vfull.push_back(*opti_it);
        } else {
          vive_vfull.push_back(*vive_it);
          opti_vfull.push_back(*(opti_it-1));
        }
        // Check if the time difference is valid
        if (abs(vive_vfull.back().header.stamp.toSec() -
          opti_vfull.back().header.stamp.toSec()) > TIME_THRESH ) {
          opti_vfull.pop_back();
          vive_vfull.pop_back();
        }
        break;
      }
    }
    if (vive_vfull.size() >= 2) {
      TF next, prev;
      // Last element
      next = vive_vfull.back();
      // Second to last element
      prev = vive_vfull[vive_vfull.size() - 2];
      // Distance between two consecutive poses
      double distance = pow(prev.transform.translation.x - next.transform.translation.x, 2) +
        pow(prev.transform.translation.y - next.transform.translation.y, 2) +
        pow(prev.transform.translation.z - next.transform.translation.z, 2);
      // Angular distance between two poses
      Eigen::Quaterniond Qnow(next.transform.rotation.w,
        next.transform.rotation.x,
        next.transform.rotation.y,
        next.transform.rotation.z);
      Eigen::Quaterniond Qpre(prev.transform.rotation.w,
        prev.transform.rotation.x,
        prev.transform.rotation.y,
        prev.transform.rotation.z);
      Eigen::AngleAxisd Adif(Qnow * Qpre.inverse());
      // Test against the thresholds
      if (distance > DISTANCE_THRESH || Adif.angle() > ANGLE_THRESH) {
        vive_v.push_back(vive_vfull.back());
        opti_v.push_back(opti_vfull.back());
      }
    // If first poses
    } else {
      vive_v.push_back(vive_vfull.back());
      opti_v.push_back(opti_vfull.back());
    }
  }

  // Estimating the offset
  TFs offset;
  offset = EstimateOffset(opti_v, vive_v);
  offset = RefineOffset(opti_vfull, vive_vfull, offset);

  rbag.close();

  // TESTING
  // TFs vive_v, opti_v, offset;
  // TestTrajectory(vive_v, opti_v);
  // offset = EstimateOffset(opti_v, vive_v);
  // offset = RefineOffset(opti_v, vive_v, offset);
  // TESTING

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