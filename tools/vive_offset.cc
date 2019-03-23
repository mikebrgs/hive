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
        T(vive_.transform.rotation.z)).toRotationMatrix();
    oPa << T(optitrack_.transform.translation.x),
        T(optitrack_.transform.translation.y),
        T(optitrack_.transform.translation.z);
    oRa << Eigen::Quaternion<T>(T(optitrack_.transform.rotation.w),
        T(optitrack_.transform.rotation.x),
        T(optitrack_.transform.rotation.y),
        T(optitrack_.transform.rotation.z)).toRotationMatrix();
    oPv << o_v[0],
        o_v[1],
        o_v[2];
    oRv << Eigen::Quaternion<T>(o_v[3],
        o_v[4],
        o_v[5],
        o_v[6]).toRotationMatrix();
    aPt << a_t[0],
        a_t[1],
        a_t[2];
    aRt << Eigen::Quaternion<T>(a_t[3],
        a_t[4],
        a_t[5],
        a_t[6]).toRotationMatrix();

    // Convert Vive to OptiTrack
    Eigen::Matrix<T, 3, 1> tPa = -aRt.transpose() * aPt;
    Eigen::Matrix<T, 3, 3> tRa = aRt.transpose();

    Eigen::Matrix<T, 3, 1> _vPa = vRt * tPa + vPt;
    Eigen::Matrix<T, 3, 3> _vRa = vRt * tRa;

    Eigen::Matrix<T, 3, 1> _oPa = oRv * _vPa + oPv;
    Eigen::Matrix<T, 3, 3> _oRa = oRv * _vRa;

    Eigen::Quaternion<T> oQa(oRa);
    Eigen::Quaternion<T> _oQa(_oRa);

    T delta = T(vive_.header.stamp.toSec()) - T(optitrack_.header.stamp.toSec());
    residual[0] = (_oPa(0) - oPa(0)) / (T(1) + delta);
    residual[1] = (_oPa(1) - oPa(1)) / (T(1) + delta);
    residual[2] = (_oPa(2) - oPa(2)) / (T(1) + delta);

    // residual[3] = acos(((_oRa * oRa.transpose()).trace() - T(1)) * T(0.5));
    // residual[3] = T(2) * cos(oQa.w() * _oQa.w() +
    //   oQa.x() * _oQa.x() +
    //   oQa.y() * _oQa.y() +
    //   oQa.z() * _oQa.z());

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

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = true;

  double oTv_[7] = {0,0,0,1,0,0,0};
  double aTt_[7] = {0,0,0,1,0,0,0};
  auto o_it = optitrack.begin();
  auto o_pit = o_it;
  auto v_it = vive.begin();

  std::cout << "oTv: "
            << oTv_[0] << ", "
            << oTv_[1] << ", "
            << oTv_[2] << ", "
            << oTv_[3] << ", "
            << oTv_[4] << ", "
            << oTv_[5] << ", "
            << oTv_[6] << std::endl;
  std::cout << "aTt: "
            << aTt_[0] << ", "
            << aTt_[1] << ", "
            << aTt_[2] << ", "
            << aTt_[3] << ", "
            << aTt_[4] << ", "
            << aTt_[5] << ", "
            << aTt_[6] << std::endl;

  while (v_it->header.stamp.toSec() < o_it->header.stamp.toSec()
    && v_it != vive.end()) {
    v_it++;
  }
  for ( ;v_it != vive.end(); v_it++) {
    while(o_it < v_it) {
      // TODO check this
      // if (o_it == optitrack.end()) {
      //   goto end_optitrack;
      // }
      o_pit = o_it;
      o_it++;
    }
    // New residual block for posterior pose
    ceres::CostFunction * cost_prev =
      new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
      (new PoseCostFunctor(*v_it, *o_pit));
    problem.AddResidualBlock(cost_prev, NULL, oTv_, aTt_);
    // New residual block for posterior pose
    ceres::CostFunction * cost_next =
      new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
      (new PoseCostFunctor(*v_it, *o_it));
    problem.AddResidualBlock(cost_next, NULL, oTv_, aTt_);
  }
end_optitrack:

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  std::cout << "oTv: "
            << oTv_[0] << ", "
            << oTv_[1] << ", "
            << oTv_[2] << ", "
            << oTv_[3] << ", "
            << oTv_[4] << ", "
            << oTv_[5] << ", "
            << oTv_[6] << std::endl;
  std::cout << "aTt: "
            << aTt_[0] << ", "
            << aTt_[1] << ", "
            << aTt_[2] << ", "
            << aTt_[3] << ", "
            << aTt_[4] << ", "
            << aTt_[5] << ", "
            << aTt_[6] << std::endl;

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
    if (solver->GetTransform(msg)) {
      vv_.push_back(msg);
    }
  }
  ROS_INFO("Data processment complete.");

  TFs offset;
  offset = ComputeOffset(otv_, vv_);

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

 /*
  *
  *  TESTING
  *
  */
  // geometry_msgs::TransformStamped tv1, tv2, tv3, tv4, to1, to2, to3, to4;
  // geometry_msgs::TransformStamped tv5, tv6, to5, to6;
  // tv1.transform.translation.x = 0.0;
  // tv1.transform.translation.y = 0.0;
  // tv1.transform.translation.z = 0.0;
  // tv1.transform.rotation.w = 1.0;
  // tv1.transform.rotation.x = 0.0;
  // tv1.transform.rotation.y = 0.0;
  // tv1.transform.rotation.z = 0.0;
  // to1 = tv1;

  // tv2.transform.translation.x = 0.0;
  // tv2.transform.translation.y = 0.0;
  // tv2.transform.translation.z = 2.0;
  // tv2.transform.rotation.w = 1.0;
  // tv2.transform.rotation.x = 0.0;
  // tv2.transform.rotation.y = 0.0;
  // tv2.transform.rotation.z = 0.0;
  // to2 = tv2;

  // tv3.transform.translation.x = 0.0;
  // tv3.transform.translation.y = 0.0;
  // tv3.transform.translation.z = 2.0;
  // tv3.transform.rotation.w = 1.0;
  // tv3.transform.rotation.x = 0.0;
  // tv3.transform.rotation.y = 0.0;
  // tv3.transform.rotation.z = 0.0;
  // to3 = tv3;

  // tv4.transform.translation.x = -1.0;
  // tv4.transform.translation.y = 1.0;
  // tv4.transform.translation.z = 0.0;
  // tv4.transform.rotation.w = 0.0;
  // tv4.transform.rotation.x = -1.0;
  // tv4.transform.rotation.y = 0.0;
  // tv4.transform.rotation.z = 0.0;
  // to4 = tv4;

  // tv5.transform.translation.x = -1.0;
  // tv5.transform.translation.y = 3.5;
  // tv5.transform.translation.z = -2.1;
  // tv5.transform.rotation.w = 0.0;
  // tv5.transform.rotation.x = 0.0;
  // tv5.transform.rotation.y = -1.0;
  // tv5.transform.rotation.z = 0.0;
  // to5 = tv5;

  // tv6.transform.translation.x = 0.0;
  // tv6.transform.translation.y = 0.0;
  // tv6.transform.translation.z = -1.0;
  // tv6.transform.rotation.w = 0.0;
  // tv6.transform.rotation.x = 0.0;
  // tv6.transform.rotation.y = 0.0;
  // tv6.transform.rotation.z = 1.0;
  // to6 = tv6;


  // ceres::CostFunction * cost1 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv1, to1));
  // problem.AddResidualBlock(cost1, NULL, oTv_, aTt_);;

  // ceres::CostFunction * cost2 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv2, to2));
  // problem.AddResidualBlock(cost2, NULL, oTv_, aTt_);;

  // ceres::CostFunction * cost3 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv3, to3));
  // problem.AddResidualBlock(cost3, NULL, oTv_, aTt_);;

  // ceres::CostFunction * cost4 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv4, to4));
  // problem.AddResidualBlock(cost4, NULL, oTv_, aTt_);;

  // ceres::CostFunction * cost5 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv5, to5));
  // problem.AddResidualBlock(cost5, NULL, oTv_, aTt_);;

  // ceres::CostFunction * cost6 =
  //   new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 7, 7>
  //   (new PoseCostFunctor(tv6, to6));
  // problem.AddResidualBlock(cost6, NULL, oTv_, aTt_);;

  // // ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2> * cost1 =
  // //   new ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2>
  // //   (new PoseCostFunctor(tv1, to1));
  // // cost1->AddParameterBlock(7);
  // // cost1->AddParameterBlock(7);
  // // cost1->SetNumResiduals(3);
  // // problem.AddResidualBlock(cost1, NULL, oTv_, aTt_);;

  // // ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2> * cost2 =
  // //   new ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2>
  // //   (new PoseCostFunctor(tv2, to2));
  // // cost2->AddParameterBlock(7);
  // // cost2->AddParameterBlock(7);
  // // cost2->SetNumResiduals(4);
  // // problem.AddResidualBlock(cost2, NULL, oTv_, aTt_);

  // // ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2> * cost3 =
  // //   new ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2>
  // //   (new PoseCostFunctor(tv3, to3));
  // // cost3->AddParameterBlock(7);
  // // cost3->AddParameterBlock(7);
  // // cost3->SetNumResiduals(4);
  // // problem.AddResidualBlock(cost3, NULL, oTv_, aTt_);

  // // ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2> * cost4 =
  // //   new ceres::DynamicAutoDiffCostFunction<PoseCostFunctor, 2>
  // //   (new PoseCostFunctor(tv4, to4));
  // // cost4->AddParameterBlock(7);
  // // cost4->AddParameterBlock(7);
  // // cost4->SetNumResiduals(4);
  // // problem.AddResidualBlock(cost4, NULL, oTv_, aTt_);

  // std::cout << "oTv: "
  //           << oTv_[0] << ", "
  //           << oTv_[1] << ", "
  //           << oTv_[2] << ", "
  //           << oTv_[3] << ", "
  //           << oTv_[4] << ", "
  //           << oTv_[5] << ", "
  //           << oTv_[6] << std::endl;
  // std::cout << "aTt: "
  //           << aTt_[0] << ", "
  //           << aTt_[1] << ", "
  //           << aTt_[2] << ", "
  //           << aTt_[3] << ", "
  //           << aTt_[4] << ", "
  //           << aTt_[5] << ", "
  //           << aTt_[6] << std::endl;
  /*
  *
  *  TESTING
  */