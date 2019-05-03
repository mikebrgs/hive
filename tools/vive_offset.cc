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

  // // Average vive poses
  // ceres::Problem problem_vive;
  // ceres::Solver::Options options_vive;
  // ceres::Solver::Summary summary_vive;
  // // Averages pose
  // double vive_pose[6];
  // vive_pose[0] = tmp_vive_.front().transform.translation.x;
  // vive_pose[1] = tmp_vive_.front().transform.translation.y;
  // vive_pose[2] = tmp_vive_.front().transform.translation.z;
  // Eigen::Quaterniond guessQvive(tmp_vive_.front().transform.rotation.w,
  //   tmp_vive_.front().transform.rotation.z,
  //   tmp_vive_.front().transform.rotation.y,
  //   tmp_vive_.front().transform.rotation.z);
  // Eigen::AngleAxisd guessAAvive(guessQvive);
  // vive_pose[3] = guessAAvive.axis()(0) * guessAAvive.angle();
  // vive_pose[4] = guessAAvive.axis()(1) * guessAAvive.angle();
  // vive_pose[5] = guessAAvive.axis()(2) * guessAAvive.angle();

  // // Set up residuals
  // for (auto pose_it = tmp_vive_.begin();
  //   pose_it != tmp_vive_.end(); pose_it++) {
  //   std::cout << "Vive It : "
  //     << pose_it->transform.translation.x << ", "
  //     << pose_it->transform.translation.y << ", "
  //     << pose_it->transform.translation.z << ", ";
  //     Eigen::Quaterniond auxQ(pose_it->transform.rotation.w,
  //       pose_it->transform.rotation.x,
  //       pose_it->transform.rotation.y,
  //       pose_it->transform.rotation.z);
  //     Eigen::AngleAxisd auxAA(auxQ);
  //     std::cout << auxAA.angle() * auxAA.axis()(0) << ", "
  //     << auxAA.angle() * auxAA.axis()(1) << ", "
  //     << auxAA.angle() * auxAA.axis()(2) << std::endl;
  //   ceres::CostFunction * cost =
  //     new ceres::AutoDiffCostFunction<PoseAverageCost, 4, 6>
  //     (new PoseAverageCost(pose_it->transform, angle_factor_));
  //   problem_vive.AddResidualBlock(cost, new ceres::CauchyLoss(CAUCHY), vive_pose);
  // }
  // // Options and solve
  // options_vive.minimizer_progress_to_stdout = false;
  // options_vive.max_num_iterations = CERES_ITERATIONS;
  // ceres::Solve(options_vive, &problem_vive, &summary_vive);
  // std::cout << "Vive: "
  //   << vive_pose[0] << ", "
  //   << vive_pose[1] << ", "
  //   << vive_pose[2] << ", "
  //   << vive_pose[3] << ", "
  //   << vive_pose[4] << ", "
  //   << vive_pose[5] << std::endl;
  // std::cout << summary_vive.BriefReport() << std::endl;

  // // Save vive pose
  // TF avg_vive;
  // avg_vive.header.frame_id = tmp_vive_.front().header.frame_id;
  // avg_vive.child_frame_id = tmp_vive_.front().child_frame_id;
  // avg_vive.header.stamp = tmp_vive_.front().header.stamp;
  // avg_vive.transform.translation.x = vive_pose[0];
  // avg_vive.transform.translation.y = vive_pose[1];
  // avg_vive.transform.translation.z = vive_pose[2];
  // Eigen::Vector3d avgVvive(vive_pose[3],
  //   vive_pose[4],
  //   vive_pose[5]);
  // Eigen::Quaterniond avgQvive;
  // if (avgVvive.norm() != 0) {
  //   Eigen::AngleAxisd avgAAvive(avgVvive.norm(),
  //     avgVvive.normalized());
  //   avgQvive = Eigen::Quaterniond(avgAAvive);
  // } else {
  //   avgQvive = Eigen::Quaterniond(1,0,0,0);
  // }
  // avg_vive.transform.rotation.w = avgQvive.w();
  // avg_vive.transform.rotation.x = avgQvive.x();
  // avg_vive.transform.rotation.y = avgQvive.y();
  // avg_vive.transform.rotation.z = avgQvive.z();

  // vive_.push_back(avg_vive);
  vive_.push_back(tmp_vive_.back());

  tmp_vive_.clear();

  // OptiTrack average poses
  // ceres::Problem problem_optitrack;
  // ceres::Solver::Options options_optitrack;
  // ceres::Solver::Summary summary_optitrack;
  // // Averages pose
  // double optitrack_pose[6];
  // optitrack_pose[0] = tmp_optitrack_.front().transform.translation.x;
  // optitrack_pose[1] = tmp_optitrack_.front().transform.translation.y;
  // optitrack_pose[2] = tmp_optitrack_.front().transform.translation.z;
  // Eigen::Quaterniond guessQoptitrack(tmp_optitrack_.front().transform.rotation.w,
  //   tmp_optitrack_.front().transform.rotation.z,
  //   tmp_optitrack_.front().transform.rotation.y,
  //   tmp_optitrack_.front().transform.rotation.z);
  // Eigen::AngleAxisd guessAAoptitrack(guessQoptitrack);
  // optitrack_pose[3] = guessAAoptitrack.axis()(0) * guessAAoptitrack.angle();
  // optitrack_pose[4] = guessAAoptitrack.axis()(1) * guessAAoptitrack.angle();
  // optitrack_pose[5] = guessAAoptitrack.axis()(2) * guessAAoptitrack.angle();

  // // Set up residuals
  // for (auto pose_it = tmp_optitrack_.begin();
  //   pose_it != tmp_optitrack_.end(); pose_it++) {
  //   std::cout << "Opti It : "
  //     << pose_it->transform.translation.x << ", "
  //     << pose_it->transform.translation.y << ", "
  //     << pose_it->transform.translation.z << ", ";
  //     Eigen::Quaterniond auxQ(pose_it->transform.rotation.w,
  //       pose_it->transform.rotation.x,
  //       pose_it->transform.rotation.y,
  //       pose_it->transform.rotation.z);
  //     Eigen::AngleAxisd auxAA(auxQ);
  //     std::cout << auxAA.angle() * auxAA.axis()(0) << ", "
  //     << auxAA.angle() * auxAA.axis()(1) << ", "
  //     << auxAA.angle() * auxAA.axis()(2) << std::endl;
  //   ceres::CostFunction * cost =
  //     new ceres::AutoDiffCostFunction<PoseAverageCost, 4, 6>
  //     (new PoseAverageCost(pose_it->transform, angle_factor_));
  //   problem_optitrack.AddResidualBlock(cost, new ceres::CauchyLoss(CAUCHY), optitrack_pose);
  // }
  // // Options and solve
  // options_optitrack.minimizer_progress_to_stdout = false;
  // options_optitrack.max_num_iterations = CERES_ITERATIONS;
  // ceres::Solve(options_optitrack, &problem_optitrack, &summary_optitrack);
  // std::cout << "Optitrack: " 
  //   << optitrack_pose[0] << ", "
  //   << optitrack_pose[1] << ", "
  //   << optitrack_pose[2] << ", "
  //   << optitrack_pose[3] << ", "
  //   << optitrack_pose[4] << ", "
  //   << optitrack_pose[5] << std::endl;
  // std::cout << summary_optitrack.BriefReport() << std::endl;

  // // Save optitrack pose
  // TF avg_optitrack;
  // avg_optitrack.header.frame_id = tmp_optitrack_.front().header.frame_id;
  // avg_optitrack.child_frame_id = tmp_optitrack_.front().child_frame_id;
  // avg_optitrack.header.stamp = tmp_optitrack_.front().header.stamp;
  // avg_optitrack.transform.translation.x = optitrack_pose[0];
  // avg_optitrack.transform.translation.y = optitrack_pose[1];
  // avg_optitrack.transform.translation.z = optitrack_pose[2];
  // Eigen::Vector3d avgVoptitrack(optitrack_pose[3],
  //   optitrack_pose[4],
  //   optitrack_pose[5]);
  // Eigen::Quaterniond avgQoptitrack;
  // if (avgVoptitrack.norm() != 0) {
  //   Eigen::AngleAxisd avgAAoptitrack(avgVoptitrack.norm(),
  //     avgVoptitrack.normalized());
  //   avgQoptitrack = Eigen::Quaterniond(avgAAoptitrack);
  // } else {
  //   avgQoptitrack = Eigen::Quaterniond(1,0,0,0);
  // }
  // avg_optitrack.transform.rotation.w = avgQoptitrack.w();
  // avg_optitrack.transform.rotation.x = avgQoptitrack.x();
  // avg_optitrack.transform.rotation.y = avgQoptitrack.y();
  // avg_optitrack.transform.rotation.z = avgQoptitrack.z();

  // optitrack_.push_back(avg_optitrack);
  optitrack_.push_back(tmp_optitrack_.back());

  tmp_optitrack_.clear();

  return true;
}

bool HiveOffset::GetOffset(TFs & offsets) {
  // If the data was collected in steps
  TFs vive_vfull, opti_vfull;
  if (steps_) {
    NextPose();
    offsets = CeresEstimateOffset(optitrack_, vive_);
    // offsets = VispEstimateOffset(optitrack_, vive_);
    // if (refine_)
    //   offsets = RefineOffset(optitrack_, vive_, offsets);
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
    offsets = CeresEstimateOffset(optitrack_, vive_);
    // offsets = VispEstimateOffset(optitrack_, vive_);
    // if (refine_)
    //   offsets = RefineOffset(optitrack_, vive_, offsets);
  }

  // Clear the data
  tmp_vive_.clear();
  vive_.clear();
  tmp_optitrack_.clear();
  optitrack_.clear();

  return true;
}

TFs HiveOffset::CeresEstimateOffset(TFs& optitrack, TFs& vive) {
  TFs offsets;
  double best_cost;
  Eigen::Matrix3d best_aRt;

  if (optitrack.size() != vive.size()) return offsets;
  if (optitrack.size() < 3) return offsets;

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Orientation optimization
  std::cout << "Offset Orientation ";
  // Random generator
  std::uniform_real_distribution<double> unif_rot(-M_PI,M_PI);
  std::default_random_engine re_rot;
  // Cycle through multiple poses
  best_cost = 9e9;
  double best_aAt[3];
  for (size_t i = 0; i < 50; i++) {
    ceres::Problem problem1;

    // Warm start
    double aAt[3];
    for (size_t j = 0; j < 3; j++) {
      aAt[j] = unif_rot(re_rot) / sqrt(3*pow(M_PI,2));
    }

    auto prev_opti_it = optitrack.begin();
    auto next_opti_it = prev_opti_it + 1;
    auto prev_vive_it = vive.begin();
    auto next_vive_it = prev_vive_it + 1;
    while (next_vive_it != vive.end()) {
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<OrientationCostFunctor, 1, 3>
        (new OrientationCostFunctor(prev_vive_it->transform.rotation,
          next_vive_it->transform.rotation,
          prev_opti_it->transform.rotation,
          next_opti_it->transform.rotation));
      problem1.AddResidualBlock(cost, NULL, aAt);
      // Next poses
      next_vive_it++;
      prev_vive_it++;
      next_opti_it++;
      prev_opti_it++;
    }

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 1000;
    ceres::Solve(options, &problem1, &summary);


    if (summary.final_cost < best_cost) {
      best_cost = summary.final_cost;
      best_aAt[0] = aAt[0];
      best_aAt[1] = aAt[1];
      best_aAt[2] = aAt[2];
    }
  }
  // Custo final da orientação
  std::cout << summary.final_cost << std::endl;

  // Full pose optimization
  std::cout << "Offset Pose" << std::endl;
  // Random uniform generator
  std::uniform_real_distribution<double> unif_pose(-0.2,0.2);
  std::default_random_engine re_pose;
  // Cycle through multiple poses
  best_cost = 9e9;
  double best_aTt[6];
  for (size_t i = 0; i < 30; i++) {
    double aTt[6];
    ceres::Problem problem2;

    aTt[0] = unif_pose(re_pose);
    aTt[1] = unif_pose(re_pose);
    aTt[2] = unif_pose(re_pose);
    aTt[3] = best_aAt[0];
    aTt[4] = best_aAt[1];
    aTt[5] = best_aAt[2];

    auto prev_opti_it = optitrack.begin();
    auto next_opti_it = prev_opti_it + 1;
    auto prev_vive_it = vive.begin();
    auto next_vive_it = prev_vive_it + 1;
    while (next_vive_it != vive.end()) {
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<HandEyeCostFunctor, 4, 6>
        (new HandEyeCostFunctor(prev_vive_it->transform,
          next_vive_it->transform,
          prev_opti_it->transform,
          next_opti_it->transform));
      problem2.AddResidualBlock(cost, NULL, aTt);
      // Next poses
      next_vive_it++;
      prev_vive_it++;
      next_opti_it++;
      prev_opti_it++;
    }


    ceres::Solve(options, &problem2, &summary);
    // Check if this is the best solution yet
    if (summary.final_cost < best_cost) {
      best_cost = summary.final_cost;
      best_aTt[0] = aTt[0];
      best_aTt[1] = aTt[1];
      best_aTt[2] = aTt[2];
      best_aTt[3] = aTt[3];
      best_aTt[4] = aTt[4];
      best_aTt[5] = aTt[5];
    }
  }

  // Printing the final solution
  std::cout << "aTt: "
    << best_cost <<  " - "
    << best_aTt[0] << ", "
    << best_aTt[1] << ", "
    << best_aTt[2] << ", "
    << best_aTt[3] << ", "
    << best_aTt[4] << ", "
    << best_aTt[5] << std::endl;

  // Compute oTv
  best_cost = 9e9;
  double best_oTv[6];
  for (size_t i = 0; i < 5; i++) {
    ceres::Problem problem3;

    double aTt[6];
    aTt[0] = best_aTt[0];
    aTt[1] = best_aTt[1];
    aTt[2] = best_aTt[2];
    aTt[3] = best_aTt[3];
    aTt[4] = best_aTt[4];
    aTt[5] = best_aTt[5];

    double oTv[6];
    oTv[0] = unif_pose(re_pose);
    oTv[1] = unif_pose(re_pose);
    oTv[2] = unif_pose(re_pose);
    oTv[3] = unif_rot(re_rot);
    oTv[4] = unif_rot(re_rot);
    oTv[5] = unif_rot(re_rot);

    auto vive_it = vive.begin();
    auto opti_it = optitrack.begin();
    while (vive_it != vive.end() && opti_it != optitrack.end()) {
      ceres::CostFunction * thecost =
        new ceres::AutoDiffCostFunction<PoseCostFunctor, 4, 6, 6>
        (new PoseCostFunctor(vive_it->transform, opti_it->transform));
      problem3.AddResidualBlock(thecost, NULL, oTv, aTt);
      vive_it++;
      opti_it++;
    }
    problem3.SetParameterBlockConstant(aTt);
    // options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 2000;
    ceres::Solve(options, &problem3, &summary);

    if (summary.final_cost < best_cost) {
      best_cost = summary.final_cost;
      best_oTv[0] = oTv[0];
      best_oTv[1] = oTv[1];
      best_oTv[2] = oTv[2];
      best_oTv[3] = oTv[3];
      best_oTv[4] = oTv[4];
      best_oTv[5] = oTv[5];
      best_aTt[0] = aTt[0];
      best_aTt[1] = aTt[1];
      best_aTt[2] = aTt[2];
      best_aTt[3] = aTt[3];
      best_aTt[4] = aTt[4];
      best_aTt[5] = aTt[5];    }
    // std::cout << summary.final_cost <<  " - "
    //   << oTv[0] << ", "
    //   << oTv[1] << ", "
    //   << oTv[2] << ", "
    //   << oTv[3] << ", "
    //   << oTv[4] << ", "
    //   << oTv[5] << std::endl;
  }

  // Printing the final solution
  std::cout << "aTt: "
    << best_cost <<  " - "
    << best_aTt[0] << ", "
    << best_aTt[1] << ", "
    << best_aTt[2] << ", "
    << best_aTt[3] << ", "
    << best_aTt[4] << ", "
    << best_aTt[5] << std::endl;

  std::cout << "oTv: "
    << best_cost <<  " - "
    << best_oTv[0] << ", "
    << best_oTv[1] << ", "
    << best_oTv[2] << ", "
    << best_oTv[3] << ", "
    << best_oTv[4] << ", "
    << best_oTv[5] << std::endl;


  // Changing the format for best_aTt
  TF aMt;
  Eigen::Vector3d aAt(best_aTt[3], best_aTt[4], best_aTt[5]);
  Eigen::AngleAxisd aAAt;
  if (aAt.norm() != 0)
    aAAt = Eigen::AngleAxisd(aAt.norm(), aAt.normalized());
  else
    aAAt = Eigen::AngleAxisd(aAt.norm(), aAt);
  Eigen::Quaterniond aQt(aAAt);
  aMt.transform.translation.x = best_aTt[0];
  aMt.transform.translation.y = best_aTt[1];
  aMt.transform.translation.z = best_aTt[2];
  aMt.transform.rotation.w = aQt.w();
  aMt.transform.rotation.x = aQt.x();
  aMt.transform.rotation.y = aQt.y();
  aMt.transform.rotation.z = aQt.z();
  aMt.header.frame_id = "arrow";
  aMt.child_frame_id = "tracker";
  // Changing the format for best_aTt
  TF oMv;
  Eigen::Vector3d oAv(best_oTv[3], best_oTv[4], best_oTv[5]);
  Eigen::AngleAxisd oAAv;
  if (oAv.norm() != 0)
    oAAv = Eigen::AngleAxisd(oAv.norm(), oAv.normalized());
  else
    oAAv = Eigen::AngleAxisd(oAv.norm(), oAv);
  Eigen::Quaterniond oQv(oAAv);
  oMv.transform.translation.x = best_oTv[0];
  oMv.transform.translation.y = best_oTv[1];
  oMv.transform.translation.z = best_oTv[2];
  oMv.transform.rotation.w = oQv.w();
  oMv.transform.rotation.x = oQv.x();
  oMv.transform.rotation.y = oQv.y();
  oMv.transform.rotation.z = oQv.z();
  oMv.header.frame_id = "optitrack";
  oMv.child_frame_id = "vive";

  offsets.push_back(aMt);
  offsets.push_back(oMv);
  return offsets;
}


TFs HiveOffset::VispEstimateOffset(TFs optitrack, TFs vive) {
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
  rosbag::Bag wbag;
  wbag.open("hive_offset.bag", rosbag::bagmode::Write);


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
    ROS_INFO("OptiTrack' setup complete.");

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
          true);
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

    for (auto of_it = offsets.begin();
      of_it != offsets.end(); of_it++) {
      wbag.write("/offset", ros::Time::now(),*of_it);
    }

  // Multiple bags - with step
  } else if (argc > 2) {
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
          std::cout << "OptiTrack: " <<
            tf_it->transform.translation.x << ", " <<
            tf_it->transform.translation.y << ", " <<
            tf_it->transform.translation.z << ", " <<
            tf_it->transform.rotation.w << ", " <<
            tf_it->transform.rotation.x << ", " <<
            tf_it->transform.rotation.y << ", " <<
            tf_it->transform.rotation.z << std::endl;
          hiver->AddOptiTrackPose(*tf_it);
          counter++;
        }
        if (counter >= 10) {
          break;
        }
      }
      ROS_INFO("OptiTrack' setup complete.");

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
            true);
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
        usleep(50000);
        // ROS_INFO("OI");
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
        if (counter >= 20 || unsuccess >= 100) {
          break;
        }
      }
      hiver->NextPose();
      rbag.close();
    }
    TFs offsets;
    ROS_INFO("OFFSET");
    hiver->GetOffset(offsets);

    for (auto of_it = offsets.begin();
      of_it != offsets.end(); of_it++) {
      wbag.write("/offset", ros::Time::now(),*of_it);
    }

  } else {
    TFs vive, optitrack, offsets;
    TestTrajectory(vive, optitrack);
    hiver = new HiveOffset(1.0, true, true);

    for (size_t i = 0; i < 10; i++) {
      hiver->AddVivePose(vive[i]);
      hiver->AddOptiTrackPose(optitrack[i]);
      hiver->NextPose();
    }

    hiver->GetOffset(offsets);

  }

  return 0;
}

void TestTrajectory(TFs & vive, TFs & optitrack) {
  std::uniform_real_distribution<double> unif(-M_PI,M_PI);
  std::default_random_engine re;
  std::normal_distribution<double> normf(0.0,0.01);

  vpTranslationVector oPv(unif(re),unif(re),unif(re));
  vpThetaUVector oAv(unif(re),unif(re),unif(re));
  vpHomogeneousMatrix oMv(oPv, oAv);
  vpTranslationVector aPt(unif(re),unif(re),unif(re));
  vpThetaUVector aAt(unif(re),unif(re),unif(re));
  std::cout << aAt[0] << ", "
    << aAt[1] << ", "
    << aAt[2] << std::endl;
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
    oTa.transform.translation.x = oPa[0] + normf(re);
    oTa.transform.translation.y = oPa[1] + normf(re);
    oTa.transform.translation.z = oPa[2] + normf(re);
    oTa.transform.rotation.w = oQa.w() + normf(re);
    oTa.transform.rotation.x = oQa.x() + normf(re);
    oTa.transform.rotation.y = oQa.y() + normf(re);
    oTa.transform.rotation.z = oQa.z() + normf(re);
    vTt.transform.translation.x = vPt[0] + normf(re);
    vTt.transform.translation.y = vPt[1] + normf(re);
    vTt.transform.translation.z = vPt[2] + normf(re);
    vTt.transform.rotation.w = vQt.w() + normf(re);
    vTt.transform.rotation.x = vQt.x() + normf(re);
    vTt.transform.rotation.y = vQt.y() + normf(re);
    vTt.transform.rotation.z = vQt.z() + normf(re);
    vive.push_back(vTt);
    optitrack.push_back(oTa);
  }
  return;
}