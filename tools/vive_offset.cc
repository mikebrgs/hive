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
    // oRv << Eigen::Quaternion<T>(o_v[3],
    //     o_v[4],
    //     o_v[5],
    //     o_v[6]).normalized().toRotationMatrix();
    ceres::AngleAxisToRotationMatrix(&o_v[3], oRv.data());
    aPt << a_t[0],
        a_t[1],
        a_t[2];
    ceres::AngleAxisToRotationMatrix(&a_t[3], aRt.data());
    // aRt << Eigen::Quaternion<T>(a_t[3],
    //     a_t[4],
    //     a_t[5],
    //     a_t[6]).normalized().toRotationMatrix();

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

    residual[3] = T(M_PI / 2) - ((_oRa * oRa.transpose()).trace() - T(1))
    - ((_oRa * oRa.transpose()).trace() - T(1)) *
    ((_oRa * oRa.transpose()).trace() - T(1))*
    ((_oRa * oRa.transpose()).trace() - T(1)) / T(6.0);
    // residual[3] = T(1.0) - (oQa.w() * _oQa.w() +
    //   oQa.x() * _oQa.x() +
    //   oQa.y() * _oQa.y() +
    //   oQa.z() * _oQa.z());
    // T x = (oQa.w() * _oQa.w() +
    //   oQa.x() * _oQa.x() +
    //   oQa.y() * _oQa.y() +
    //   oQa.z() * _oQa.z());
    // residual[3] = T(M_PI/2) - x - x*x*x / T(6.0);

    return true;
  }
 private:
  TF vive_;
  TF optitrack_;
};

TFs RefineOffsets(TFs optitrack, TFs vive, TFs offset) {

  TF aTt = offset[0]; // Transform from tracker to optitrack's arrow
  TF oTv = offset[1]; // Transform from vive to optitrack


  // TODO Continue here
  vpTranslationVector tPa;
  vpThetaUVector tRa;
  tMa.inverse().extract(tPa);
  tMa.inverse().extract(tRa);

  double tTa[6] = {
    tPa[0],
    tPa[1],
    tPa[2],
    tRa[0],
    tRa[1],
    tRa[2],
  };

  double vTo[6] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
  };

  ceres::Problem problem;

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 50;
  // options.parameter_tolerance = 1e-40;
  // options.gradient_tolerance = 1e-40;
  // options.function_tolerance = 1e-40;
  // options.minimizer_type = ceres::LINE_SEARCH;
  // options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
  // options.trust_region_strategy_type = ceres::DOGLEG;
  // options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  auto o_it = optitrack.begin();
  auto o_pit = o_it;
  auto v_it = vive.begin();

  // Different notation
  std::cout << "oTv: "
            << vTo[0] << ", "
            << vTo[1] << ", "
            << vTo[2] << ", "
            << vTo[3] << ", "
            << vTo[4] << ", "
            << vTo[5] << std::endl;
  std::cout << "aTt: "
            << tTa[0] << ", "
            << tTa[1] << ", "
            << tTa[2] << ", "
            << tTa[3] << ", "
            << tTa[4] << ", "
            << tTa[5] << std::endl;

  while(v_it != vive.end()) {
    ceres::CostFunction * cost_prev =
      new ceres::AutoDiffCostFunction<PoseCostFunctor, 4, 6, 6>
      (new PoseCostFunctor(*v_it, *o_it));
    problem.AddResidualBlock(cost_prev, NULL, vTo, tTa);
    v_it++;
    o_it++;
  }

  // while (v_it->header.stamp.toSec() < o_it->header.stamp.toSec()
  //   && v_it != vive.end()) {
  //   v_it++;
  // }
  // for ( ;v_it != vive.end(); v_it++) {
  //   while(o_it < v_it && o_it == optitrack.end()) {
  //     o_pit = o_it;
  //     o_it++;
  //   }
  //   if (o_it == optitrack.end()) break;
  //   // New residual block for posterior pose
  //   ceres::CostFunction * cost_prev =
  //     new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //     (new PoseCostFunctor(*v_it, *o_pit));
  //   problem.AddResidualBlock(cost_prev, NULL, oTv_, aTt_);
  //   // New residual block for posterior pose
  //   ceres::CostFunction * cost_next =
  //     new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //     (new PoseCostFunctor(*v_it, *o_it));
  //   problem.AddResidualBlock(cost_next, NULL, oTv_, aTt_);
  // }

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  std::cout << "oTv: "
            << vTo[0] << ", "
            << vTo[1] << ", "
            << vTo[2] << ", "
            << vTo[3] << ", "
            << vTo[4] << ", "
            << vTo[5] << std::endl;
  std::cout << "aTt: "
            << tTa[0] << ", "
            << tTa[1] << ", "
            << tTa[2] << ", "
            << tTa[3] << ", "
            << tTa[4] << ", "
            << tTa[5] << std::endl;

  offset.clear();
  // TODO Replace transforms
  return offset;
}

TFs EstimateOffset(TFs optitrack, TFs vive) {
  std::vector<vpHomogeneousMatrix> vMtVec; // camera to object - tracker to vive
  std::vector<vpHomogeneousMatrix> tMvVec;
  std::vector<vpHomogeneousMatrix> aMoVec; // reference to end effector - optitrack to arrow
  std::vector<vpHomogeneousMatrix> oMaVec;
  vpHomogeneousMatrix tMa; // end effector to camera - arrow to tracker
  vpHomogeneousMatrix vMo; // TODO Check this one

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
    // vMt.push_back(vMt); // To compute the big offset
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
    // aMo.push_back(oMa.inverse()); // To compute the big offset
    oMaVec.push_back(oMa); // To compute the small offset
}

  int status;

  status = vpHandEyeCalibration::calibrate(tMvVec, oMaVec, tMa); // To compute the small offset
  std::cout << "tMa: ";
  tMa.print(); // end effector to camera - arrow to tracker
  std::cout << std::endl << "Status: " <<  status << std::endl;
  // TODO check this
  status = vpHandEyeCalibration::calibrate(tMvVec, oMaVec, vMo); // To compute the small offset
  std::cout << "vMo: ";
  vMo.print(); // end effector to camera - arrow to tracker
  std::cout << std::endl << "Status: " <<  status << std::endl;

  TFs Ts;
  TF aTt; // Transform from tracker to optitrack's arrow
  TF oTv; // Transform from vive to optitrack

  // Changing the format for aTt
  vpTranslationVector aPt;
  vpQuaternionVector aQt;
  tMa.inverse().extract(aPt);
  tMa.inverse().extract(aQt);
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
  oMv.inverse().extract(oPv);
  oMv.inverse().extract(oQv);
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
    if (solver->GetTransform(msg)) {
      vv_.push_back(msg);
    }
  }
  ROS_INFO("Data processment complete.");

  // TFs offset;
  // offset = EstimateOffset(otv_, vv_);

  TFs vive_v, opti_v;
  vive_v.push_back(*vv_.begin());
  for (size_t msg_it = 1; msg_it < vv_.size(); msg_it++) {
    double distance = pow(vv_[msg_it].transform.translation.x - vive_v.back().transform.translation.x, 2) +
      pow(vv_[msg_it].transform.translation.y - vive_v.back().transform.translation.y, 2) +
      pow(vv_[msg_it].transform.translation.z - vive_v.back().transform.translation.z, 2);
    if (distance > 0.1) {
    // if (msg_it % (vv_.size()/20) == 0) {
      vive_v.push_back(vv_[msg_it]);
      std::cout << msg_it <<  ": " << vive_v.back().transform.translation.x << ", " <<
      vive_v.back().transform.translation.y << ", " <<
      vive_v.back().transform.translation.z << std::endl;
    }
  }
  vive_v.erase(vive_v.begin());
  vive_v.erase(vive_v.begin());
  vive_v.pop_back();

  std::cout << "***" << std::endl;

  size_t vi_it = 0;
  for (size_t op_it = 0 ; op_it < otv_.size(); op_it++) {
    // std::cout << "HERE1" << std::endl;
    if (otv_[op_it].header.stamp.toSec() > vive_v[vi_it].header.stamp.toSec()) {
      if (otv_[op_it].header.stamp.toSec() - vive_v[vi_it].header.stamp.toSec() <
        - otv_[op_it - 1].header.stamp.toSec() + vive_v[vi_it].header.stamp.toSec()) {
        opti_v.push_back(otv_[op_it]);
      } else {
        opti_v.push_back(otv_[op_it - 1]);
      }
      std::cout << op_it <<  ": " << opti_v.back().transform.translation.x << ", " <<
      opti_v.back().transform.translation.y << ", " <<
      opti_v.back().transform.translation.z << std::endl;
      vi_it++;
      if (vi_it >= vive_v.size()) break;
    }
  }


  TFs offset;
  offset = EstimateOffset(opti_v, vive_v);
  // offset = RefineOffsets(opti_v, vive_v, offset);


  // Eigen::Quaterniond Q(0.1,10.0,0.0,0.0);
  // Q.normalize();
  // std::cout << Q.toRotationMatrix() << std::endl;

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