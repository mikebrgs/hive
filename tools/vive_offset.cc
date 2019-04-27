/*
 * @version 1.0
 * @author Miguel Rego Borges
 * Instituto Superior Tecnico - University of Lisbon
 * Purpose: calculate offset between vive's frame and optitrack's
*/

#include <hive/vive_offset.h>

HiveOffset::HiveOffset(double angle_factor, bool refine, bool steps) {
  steps_ = steps;
  refine_ = refine;
  angle_factor_ = angle_factor;
  return;
}

HiveOffset::~HiveOffset() {
  
}

bool HiveOffset::AddVivePose(const TF& pose) {
  tmp_vive_.push_back(pose);
  return true;
}

bool HiveOffset::AddOptiTrackPose(const TF& pose) {
  tmp_optitrack_.push_back(pose);
  return true;
}

bool HiveOffset::NextPose() {
  // Check object type
  if (!steps_) return true;

  // Check if for any vector the number of poses is zero
  if (tmp_vive_.size() == 0 || tmp_optitrack_.size() == 0) {
    return true;
  }

  // Average vive poses
  ceres::Problem problem_vive;
  ceres::Solver::Options options_vive;
  ceres::Solver::Summary summary_vive;
  // Averages pose
  double vive_pose[6];
  vive_pose[0] = tmp_vive_.front().transform.translation.x;
  vive_pose[1] = tmp_vive_.front().transform.translation.y;
  vive_pose[2] = tmp_vive_.front().transform.translation.z;
  Eigen::Quaterniond guessQvive(tmp_vive_.front().transform.rotation.w,
    tmp_vive_.front().transform.rotation.z,
    tmp_vive_.front().transform.rotation.y,
    tmp_vive_.front().transform.rotation.z);
  Eigen::AngleAxisd guessAAvive(guessQvive);
  vive_pose[3] = guessAAvive.axis()(0) * guessAAvive.angle();
  vive_pose[4] = guessAAvive.axis()(1) * guessAAvive.angle();
  vive_pose[5] = guessAAvive.axis()(2) * guessAAvive.angle();

  // Set up residuals
  for (auto pose_it = tmp_vive_.begin();
    pose_it != tmp_vive_.end(); pose_it++) {
    ceres::CostFunction * cost =
      new ceres::AutoDiffCostFunction<PoseAverageCost, 4, 6>
      (new PoseAverageCost(pose_it->transform, angle_factor_));
    problem_vive.AddResidualBlock(cost, new ceres::CauchyLoss(CAUCHY), vive_pose);
  }
  // Options and solve
  options_vive.minimizer_progress_to_stdout = false;
  options_vive.max_num_iterations = CERES_ITERATIONS;
  ceres::Solve(options_vive, &problem_vive, &summary_vive);
  std::cout << "Vive: "
    << vive_pose[0] << ", "
    << vive_pose[1] << ", "
    << vive_pose[2] << ", "
    << vive_pose[3] << ", "
    << vive_pose[4] << ", "
    << vive_pose[5] << std::endl;
  std::cout << summary_vive.BriefReport() << std::endl;

  // Save vive pose
  TF avg_vive;
  avg_vive.header.frame_id = tmp_vive_.front().header.frame_id;
  avg_vive.child_frame_id = tmp_vive_.front().child_frame_id;
  avg_vive.header.stamp = tmp_vive_.front().header.stamp;
  avg_vive.transform.translation.x = vive_pose[0];
  avg_vive.transform.translation.y = vive_pose[1];
  avg_vive.transform.translation.z = vive_pose[2];
  Eigen::Vector3d avgVvive(vive_pose[3],
    vive_pose[4],
    vive_pose[5]);
  Eigen::Quaterniond avgQvive;
  if (avgVvive.norm() != 0) {
    Eigen::AngleAxisd avgAAvive(avgVvive.norm(),
      avgVvive.normalized());
    avgQvive = Eigen::Quaterniond(avgAAvive);
  } else {
    avgQvive = Eigen::Quaterniond(1,0,0,0);
  }
  avg_vive.transform.rotation.w = avgQvive.w();
  avg_vive.transform.rotation.x = avgQvive.x();
  avg_vive.transform.rotation.y = avgQvive.y();
  avg_vive.transform.rotation.z = avgQvive.z();
  vive_.push_back(avg_vive);

  tmp_vive_.clear();

  // OptiTrack average poses
  ceres::Problem problem_optitrack;
  ceres::Solver::Options options_optitrack;
  ceres::Solver::Summary summary_optitrack;
  // Averages pose
  double optitrack_pose[6];
  optitrack_pose[0] = tmp_optitrack_.front().transform.translation.x;
  optitrack_pose[1] = tmp_optitrack_.front().transform.translation.y;
  optitrack_pose[2] = tmp_optitrack_.front().transform.translation.z;
  Eigen::Quaterniond guessQoptitrack(tmp_optitrack_.front().transform.rotation.w,
    tmp_optitrack_.front().transform.rotation.z,
    tmp_optitrack_.front().transform.rotation.y,
    tmp_optitrack_.front().transform.rotation.z);
  Eigen::AngleAxisd guessAAoptitrack(guessQoptitrack);
  optitrack_pose[3] = guessAAoptitrack.axis()(0) * guessAAoptitrack.angle();
  optitrack_pose[4] = guessAAoptitrack.axis()(1) * guessAAoptitrack.angle();
  optitrack_pose[5] = guessAAoptitrack.axis()(2) * guessAAoptitrack.angle();

  // Set up residuals
  for (auto pose_it = tmp_optitrack_.begin();
    pose_it != tmp_optitrack_.end(); pose_it++) {
    ceres::CostFunction * cost =
      new ceres::AutoDiffCostFunction<PoseAverageCost, 4, 6>
      (new PoseAverageCost(pose_it->transform, angle_factor_));
    problem_optitrack.AddResidualBlock(cost, new ceres::CauchyLoss(CAUCHY), optitrack_pose);
  }
  // Options and solve
  options_optitrack.minimizer_progress_to_stdout = false;
  options_optitrack.max_num_iterations = CERES_ITERATIONS;
  ceres::Solve(options_optitrack, &problem_optitrack, &summary_optitrack);
  std::cout << "Optitrack: " 
    << optitrack_pose[0] << ", "
    << optitrack_pose[1] << ", "
    << optitrack_pose[2] << ", "
    << optitrack_pose[3] << ", "
    << optitrack_pose[4] << ", "
    << optitrack_pose[5] << std::endl;
  std::cout << summary_optitrack.BriefReport() << std::endl;

  // Save optitrack pose
  TF avg_optitrack;
  avg_optitrack.header.frame_id = tmp_optitrack_.front().header.frame_id;
  avg_optitrack.child_frame_id = tmp_optitrack_.front().child_frame_id;
  avg_optitrack.header.stamp = tmp_optitrack_.front().header.stamp;
  avg_optitrack.transform.translation.x = optitrack_pose[0];
  avg_optitrack.transform.translation.y = optitrack_pose[1];
  avg_optitrack.transform.translation.z = optitrack_pose[2];
  Eigen::Vector3d avgVoptitrack(optitrack_pose[3],
    optitrack_pose[4],
    optitrack_pose[5]);
  Eigen::Quaterniond avgQoptitrack;
  if (avgVoptitrack.norm() != 0) {
    Eigen::AngleAxisd avgAAoptitrack(avgVoptitrack.norm(),
      avgVoptitrack.normalized());
    avgQoptitrack = Eigen::Quaterniond(avgAAoptitrack);
  } else {
    avgQoptitrack = Eigen::Quaterniond(1,0,0,0);
  }
  avg_optitrack.transform.rotation.w = avgQoptitrack.w();
  avg_optitrack.transform.rotation.x = avgQoptitrack.x();
  avg_optitrack.transform.rotation.y = avgQoptitrack.y();
  avg_optitrack.transform.rotation.z = avgQoptitrack.z();
  optitrack_.push_back(avg_optitrack);

  tmp_optitrack_.clear();

  return true;
}

bool HiveOffset::GetOffset(TFs & offsets) {
  // If the data was collected in steps
  TFs vive_vfull, opti_vfull;
  if (steps_) {
    NextPose();
    offsets = EstimateOffset(optitrack_, vive_);
    if (refine_)
      offsets = RefineOffset(optitrack_, vive_, offsets);
  // If we have continuous data
  } else {
    // Search the vive poses
    for (auto vive_it = tmp_vive_.begin();
      vive_it != tmp_vive_.end(); vive_it++) {
      // Search the optitrack poses
      for (auto opti_it = tmp_optitrack_.begin();
        opti_it != tmp_optitrack_.end(); opti_it++) {
        // Check if the optitrack iterator is after the vive pose
        if (opti_it->header.stamp.toSec() > vive_it->header.stamp.toSec()) {
          // Find the closes image
          if (opti_it == tmp_optitrack_.begin()) {
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
          vive_.push_back(vive_vfull.back());
          optitrack_.push_back(opti_vfull.back());
        }
      // If first poses
      } else {
        vive_.push_back(vive_vfull.back());
        optitrack_.push_back(opti_vfull.back());
      }
    }

    std::cout << vive_.size() << " " << optitrack_.size() << std::endl;
    // Estimating the offset
    offsets = EstimateOffset(optitrack_, vive_);
    if (refine_)
      offsets = RefineOffset(optitrack_, vive_, offsets);
  }

  // Clear the data
  tmp_vive_.clear();
  vive_.clear();
  tmp_optitrack_.clear();
  optitrack_.clear();

  return true;
}

TFs HiveOffset::EstimateOffset(TFs optitrack, TFs vive) {
  std::vector<vpHomogeneousMatrix> vMtVec; // camera to object - tracker to vive
  std::vector<vpHomogeneousMatrix> tMvVec;
  std::vector<vpHomogeneousMatrix> aMoVec; // reference to end effector - optitrack to arrow
  std::vector<vpHomogeneousMatrix> oMaVec;

  // Tracker in Vive's frame
  for (auto msg_it = vive.begin();
    msg_it != vive.end(); msg_it++) {
    vpHomogeneousMatrix * vMt = new vpHomogeneousMatrix();
    vpTranslationVector vtt(msg_it->transform.translation.x,
      msg_it->transform.translation.y,
      msg_it->transform.translation.z);
    vpQuaternionVector vrt(msg_it->transform.rotation.x,
      msg_it->transform.rotation.y,
      msg_it->transform.rotation.z,
      msg_it->transform.rotation.w);
    vMt->buildFrom(vtt, vrt);
    vMtVec.push_back(*vMt); // To compute the big offset
    tMvVec.push_back(vMt->inverse()); // To compute the small offset
  }

  // Arrow in Optitrack's frame
  for (auto msg_it = optitrack.begin();
    msg_it != optitrack.end(); msg_it++) {
    vpHomogeneousMatrix * oMa = new vpHomogeneousMatrix();
    vpTranslationVector ota(msg_it->transform.translation.x,
      msg_it->transform.translation.y,
      msg_it->transform.translation.z);
    vpQuaternionVector ora(msg_it->transform.rotation.x,
      msg_it->transform.rotation.y,
      msg_it->transform.rotation.z,
      msg_it->transform.rotation.w);
    oMa->buildFrom(ota, ora);
    aMoVec.push_back(oMa->inverse()); // To compute the big offset
    oMaVec.push_back(*oMa); // To compute the small offset
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

  TFs * Ts = new TFs();
  TF * aTt = new TF(); // Transform from tracker to optitrack's arrow
  TF * oTv = new TF(); // Transform from vive to optitrack

  // Changing the format for aTt
  vpTranslationVector aPt;
  vpQuaternionVector aQt;
  aMt.extract(aPt);
  aMt.extract(aQt);
  aTt->transform.translation.x = aPt[0];
  aTt->transform.translation.y = aPt[1];
  aTt->transform.translation.z = aPt[2];
  aTt->transform.rotation.w = aQt.w();
  aTt->transform.rotation.x = aQt.x();
  aTt->transform.rotation.y = aQt.y();
  aTt->transform.rotation.z = aQt.z();

  // Changing the format for oTv
  vpTranslationVector oPv;
  vpQuaternionVector oQv;
  oMv.extract(oPv);
  oMv.extract(oQv);
  oTv->transform.translation.x = oPv[0];
  oTv->transform.translation.y = oPv[1];
  oTv->transform.translation.z = oPv[2];
  oTv->transform.rotation.w = oQv.w();
  oTv->transform.rotation.x = oQv.x();
  oTv->transform.rotation.y = oQv.y();
  oTv->transform.rotation.z = oQv.z();

  Ts->push_back(*aTt);
  Ts->push_back(*oTv);
  return *Ts;
}

TFs HiveOffset::RefineOffset(TFs optitrack, TFs vive, TFs offset) {
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
  options.max_num_iterations = 2000;

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
      new ceres::AutoDiffCostFunction<PoseCostFunctor, 4, 6, 6>
      (new PoseCostFunctor(v_it->transform, o_it->transform));
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

int main(int argc, char ** argv) {
  ros::init(argc, argv, "hive_offset");
  ros::NodeHandle nh;

  HiveOffset * hiver;

  Calibration calibration;
  std::map<std::string, Solver*> solver;

  // Wrong parameters
  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
      << "[name_of_write_bag.bag]" << std::endl;
    return -1;
  }

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &calibration)) {
    ROS_FATAL("Can't find calibration file.");
    return false;
  } else {
    ROS_INFO("Read calibration file.");
  }

  // No step
  if (argc == 2) {
    ROS_INFO ("NO STEP OFFSET.");

    rosbag::View view;
    rosbag::Bag rbag;
    hiver = new HiveOffset(1.0, true, false);
    rbag.open(argv[1], rosbag::bagmode::Read);

    // Read OptiTrack poses
    rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
    for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
      const tf2_msgs::TFMessage::ConstPtr tf =
        bag_it->instantiate<tf2_msgs::TFMessage>();
      for (auto tf_it = tf->transforms.begin();
        tf_it != tf->transforms.end(); tf_it++) {
        hiver->AddOptiTrackPose(*tf_it);
      }
    }
    ROS_INFO("Lighthouses' setup complete.");

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
      for (auto tracker : calibration.trackers) {
        solver[tracker.first] = new ViveSolve(tracker.second,
          calibration.environment,
          calibration.lighthouses,
          false);
      }
    }
    ROS_INFO("Trackers' setup complete.");

    // Light data
    size_t counter = 0;
    rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
    for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
      const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
      solver[vl->header.frame_id]->ProcessLight(vl);
      geometry_msgs::TransformStamped msg;
      if (solver[vl->header.frame_id]->GetTransform(msg)) {
        msg.header.stamp = vl->header.stamp;
        hiver->AddVivePose(msg);
        counter++;
      }
      if (counter >= 400) break;
    }
    rbag.close();
    TFs offsets;
    ROS_INFO("Getting Offset");
    hiver->GetOffset(offsets);

  // Multiple bags - with step
  } else {
    ROS_INFO ("STEP OFFSET.");

    hiver = new HiveOffset(1.0, true, true);

    // Iterate over all bags
    size_t counter;
    for (size_t arg = 1; arg < argc; arg++) {
      rosbag::View view;
      rosbag::Bag rbag;
      rbag.open(argv[arg], rosbag::bagmode::Read);

      // Read OptiTrack poses
      counter = 0;
      rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
      for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
        const tf2_msgs::TFMessage::ConstPtr tf =
          bag_it->instantiate<tf2_msgs::TFMessage>();
        for (auto tf_it = tf->transforms.begin();
          tf_it != tf->transforms.end(); tf_it++) {
          // std::cout << "OptiTrack: " <<
          //   tf_it->transform.translation.x << ", " <<
          //   tf_it->transform.translation.y << ", " <<
          //   tf_it->transform.translation.z << ", " <<
          //   tf_it->transform.rotation.w << ", " <<
          //   tf_it->transform.rotation.x << ", " <<
          //   tf_it->transform.rotation.y << ", " <<
          //   tf_it->transform.rotation.z << std::endl;
          hiver->AddOptiTrackPose(*tf_it);
          counter++;
        }
        if (counter >= 10) break;
      }
      ROS_INFO("Lighthouses' setup complete.");

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
        for (auto tracker : calibration.trackers) {
          solver[tracker.first] = new ViveSolve(tracker.second,
            calibration.environment,
            calibration.lighthouses,
            false);
        }
      }
      ROS_INFO("Trackers' setup complete.");

      // Light data
      counter = 0;
      size_t unsuccess = 0;
      rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
      for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
        const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
        solver[vl->header.frame_id]->ProcessLight(vl);
        usleep(200000);
        geometry_msgs::TransformStamped msg;
        if (solver[vl->header.frame_id]->GetTransform(msg)) {
          msg.header.stamp = vl->header.stamp;
          hiver->AddVivePose(msg);
          // std::cout << "Vive: " <<
          //   msg.transform.translation.x << ", " <<
          //   msg.transform.translation.y << ", " <<
          //   msg.transform.translation.z << ", " <<
          //   msg.transform.rotation.w << ", " <<
          //   msg.transform.rotation.x << ", " <<
          //   msg.transform.rotation.y << ", " <<
          //   msg.transform.rotation.z << std::endl;
          counter++;
        } else {
          unsuccess++;
        }
        if (counter >= 20 || unsuccess >= 100) break;
      }
      hiver->NextPose();
      rbag.close();
    }
    TFs offsets;
    hiver->GetOffset(offsets);
  }

  return 0;
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