#include <hive/vive_refine.h>

#include <hive/hive_solve.h>

typedef geometry_msgs::TransformStamped TF;

#define SMOOTHING 0.1



// Trajectory smoothing cost function
class SmoothingCost {
public:
  // Constructor to pass data
  explicit SmoothingCost(double smoothing) {
    smoothing_ = smoothing;
  }
  // Function for ceres solver with parameters
  template <typename T> bool operator()(const T* const prev_lTt,
    const T* const prev_vTl,
    const T* const next_lTt,
    const T* const next_vTl,
    T * residual) const {
    // cost function here

    // Frame conversion
    Eigen::Matrix<T, 3, 1> prev_lPt(prev_lTt[0],
      prev_lTt[1],
      prev_lTt[2]);
    Eigen::Matrix<T, 3, 1> next_lPt(next_lTt[0],
      next_lTt[1],
      next_lTt[2]);
    Eigen::Matrix<T, 3, 1> prev_lAt(prev_lTt[3],
      prev_lTt[4],
      prev_lTt[5]);
    Eigen::Matrix<T, 3, 1> next_lAt(next_lTt[3],
      next_lTt[4],
      next_lTt[5]);
    Eigen::AngleAxis<T> prev_lAAt(prev_lAt.norm(),
      prev_lAt.normalize());
    Eigen::AngleAxis<T> next_lAAt(next_lAt.norm(),
      next_lAt.normalize());

    Eigen::Matrix<T, 3, 1> prev_vPl(prev_vTl[0],
      prev_vTl[1],
      prev_vTl[2]);
    Eigen::Matrix<T, 3, 1> next_vPl(next_vTl[0],
      next_vTl[1],
      next_vTl[2]);
    Eigen::Matrix<T, 3, 1> prev_vAl(prev_vTl[3],
      prev_vTl[4],
      prev_vTl[5]);
    Eigen::Matrix<T, 3, 1> next_vAl(next_vTl[3],
      next_vTl[4],
      next_vTl[5]);
    Eigen::AngleAxis<T> prev_vAAl(prev_vAl.norm(),
      prev_vAl.normalize());
    Eigen::AngleAxis<T> next_vAAl(next_vAl.norm(),
      next_vAl.normalize());

    Eigen::Matrix<T, 3, 1> prev_vPt = prev_vAAl * prev_lPt + prev_vPl;
    Eigen::AngleAxis<T> prev_vAAt = prev_vAAl * prev_lAAt;

    Eigen::Matrix<T, 3, 1> next_vPt = next_vAAl * next_lPt + next_vPl;
    Eigen::AngleAxis<d> next_vAAt = next_vAAl * next_vAAt;

    // Translation cost with smoothing
    residual[0] = smoothing_ * (prev_vPt(0) - next_vPt(0));
    residual[1] = smoothing_ * (prev_vPt(1) - next_vPt(1));
    residual[2] = smoothing_ * (prev_vPt(2) - next_vPt(2));
    // Rotation cost with smoothing
    residual[3] = smoothing_ * (next_vAAt.inverse() * prev_vAAt).angle();
    return true;
  }
private:
  smoothing_;
};

class PoseHorizontalCost
{
public:
  PoseHorizontalCost(hive::ViveLight data,
    Motor tracker,
    Lighthouse lighthouse,
    bool correction) {
    data_ = data;
    correction_ = correction;
    tracker_ = tracker;
    lighthouse_ = lighthouse;
  }

  template <typename T> bool operator()(const T* const parameters,
    T * residual) const {
    Eigen::Matrix<T, 3, 1> lPt(parameters[0][0],
      parameters[0][1],
      parameters[0][2]);
    Eigen::Matrix<T, 3, 1> lAt(parameters[0][3],
      parameters[0][4],
      parameters[0][5]);
    Eigen::AngleAxis<T> lAAt(lAt.norm(), lAt.normalize());

    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      Eigen::Matrix<T, 3, 1> tPs(tracker_.sensors[li_it->sensor].position.x,
        tracker_.sensors[li_it->sensor].position.y,
        tracker_.sensors[li_it->sensor].position.z);

      Eigen::Matrix<T, 3, 1> lPs = lAAt.toRotationMatrix() * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = tracker.phase;
      T tilt = tracker.tilt;
      T gib_phase = tracker.gib_phase;
      T gib_mag = tracker.gib_magnitude;
      T curve = tracker.curve;

      if (correction_) {
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }

      residual[i] = T(li_it->angle) - ang;

    }
    return true;
  }
private:
  bool correction_;
  Motor tracker_;
  Lighthouse lighthouse_;
  hive::ViveLight data_;
};

class PoseVerticalCost
{
public:
  PoseVerticalCost(hive::ViveLight data,
    Motor tracker,
    Lighthouse lighthouse,
    bool correction) {
    data_ = data;
    correction_ = correction;
    tracker_ = tracker;
    lighthouse_ = lighthouse;
  }

  template <typename T> bool operator()(const T* const parameters,
    T * residual) const {
    Eigen::Matrix<T, 3, 1> lPt(parameters[0][0],
      parameters[0][1],
      parameters[0][2]);
    Eigen::Matrix<T, 3, 1> lAt(parameters[0][3],
      parameters[0][4],
      parameters[0][5]);
    Eigen::AngleAxis<T> lAAt(lAt.norm(), lAt.normalize());

    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      Eigen::Matrix<T, 3, 1> tPs(tracker_.sensors[li_it->sensor].position.x,
        tracker_.sensors[li_it->sensor].position.y,
        tracker_.sensors[li_it->sensor].position.z);

      Eigen::Matrix<T, 3, 1> lPs = lAAt.toRotationMatrix() * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = tracker.phase;
      T tilt = tracker.tilt;
      T gib_phase = tracker.gib_phase;
      T gib_mag = tracker.gib_magnitude;
      T curve = tracker.curve;

      if (correction_) {
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }

      residual[i] = T(li_it->angle) - ang;

    }
    return true;
  }
private:
  bool correction_;
  Motor tracker_;
  Lighthouse lighthouse_;
  hive::ViveLight data_;
};

bool SolvePose(hive::ViveLight & horizontal_observations,
  hive::ViveLight & vertical_observations,
  geometry_msgs::TransformStamped & tf,
  Tracker & tracker,
  Lighthouse & lighthouse,
  bool correction) {
  // Initializations
  bool correction;
  double pose[6];

  ceres::Problem problem;

  // Data conversion
  pose[0] = pose.transform.translation.x;
  pose[1] = pose.transform.translation.y;
  pose[2] = pose.transform.translation.z;
  Eigen::Quaterniond Q(pose.transform.rotation.w,
    pose.transform.rotation.x,
    pose.transform.rotation.y,
    pose.transform.rotation.z);
  Eigen::AngleAxisd AA(Q);
  pose[3] = AA.axis()(0) * AA.angle();
  pose[4] = AA.axis()(1) * AA.angle();
  pose[5] = AA.axis()(2) * AA.angle();

  // Find if correction is being used
  if (lighthouse != NULL && CORRECTION) {
    correction = true;
  } else {
    correction = false;
  }

  ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4> * hcost =
    new ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4>
    (new PoseHorizontalCost(horizontal_observations, tracker, lighthouse, correction));
  hcost->AddParameterBlock(6);
  hcost->SetNumResiduals(horizontal_observations->samples.size());
  problem.AddResidualBlock(hcost,
    NULL,
    pose);

  ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4> * vcost =
    new ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4>
    (new PoseVerticalCost(vertical_observations, tracker, lighthouse, correction));
  vcost->AddParameterBlock(6);
  vcost->SetNumResiduals(vertical_observations->samples.size());
  problem.AddResidualBlock(vcost,
    NULL,
    pose);

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 1.0;

  ceres::Solve(options, &problem, &summary);

  // Obtain the angles again and compare the results
  {
    std::cout << "CT: " << summary.final_cost << ", " << summary.num_residual_blocks << ", " << summary.num_residuals << " - "
      << pose[0] << ", "
      << pose[1] << ", "
      << pose[2] << ", "
      << pose[3] << ", "
      << pose[4] << ", "
      << pose[5];
    if (CORRECTION) {
      std::cout << " - CORRECTED" << std::endl;
    } else {
      std::cout << " - NOT CORRECTED" << std::endl;
    }
  }

  // Check if valid
  double pose_norm = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
  if (summary.final_cost > 1e-5* 0.5 * static_cast<double>(unsigned( vertical_observations.samples.size()
    + horizontal_observations.samples.size()))
    || pose_norm > 20
    || pose[2] <= 0 ) {
    return false;
  }

  double angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  // Change the axis angle to an acceptable interval
  while (angle_norm > M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm - 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm - 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm - 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }
  // Change the axis angle to an acceptable interval
  while (angle_norm < - M_PI) {
    pose[3] = pose[3] / angle_norm * (angle_norm + 2 * M_PI);
    pose[4] = pose[4] / angle_norm * (angle_norm + 2 * M_PI);
    pose[5] = pose[5] / angle_norm * (angle_norm + 2 * M_PI);
    angle_norm = sqrt(pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5]);
  }

  // Save the solved pose
  tf.transform.translation.x = pose[0];
  tf.transform.translation.y = pose[1];
  tf.transform.translation.z = pose[2];

  Eigen::Vector3d A(pose[3], pose[4], pose[5]);
  AA = Eigen::AngleAxisd(A.norm(), A.normalize());
  Q = Eigen::Quaterniond(AA);
  tf.transform.rotation.w = Q.w();
  tf.transform.rotation.x = Q.x();
  tf.transform.rotation.y = Q.y();
  tf.transform.rotation.z = Q.z();

  tf.header.frame_id = lighthouse.serial;
  tf.child_frame_id = tracker.serial;

  return true;
}

Refinery::Refinery(Calibration & calibration) {
  // Initialize calibration (reference)
  calibration_ = calibration;

  return;
}

Refinery::~Refinery() {
  // pass
}

bool Refinery::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  return true; // nothing for now
  if (msg == NULL) return false;

  data_[msg->header.frame_id].second.push_back(*msg);

  return true;
}

bool Refinery::AddLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return false;

  data_[msg->header.frame_id].first.push_back(*msg);

  return true;
}

// Solve the problem
bool Refinery::Solve() {
  // Check requirements
  if (calibration_.environment.lighthouses.size() <= 1)
    ROS_WARN("Not enough lighthouses for Refinement.");

  // Ceres set up
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Options::Summary summary;
  // Solver's options
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 200;

  // Environment transforms
  std::map<std::string, double[6]> vTl;
  for (auto lh_it = calibration_.environment.lighthouses.begin();
    lh_it != calibration_.environment.lighthouses.end(); lh_it++) {
    // Conversion from quaternion to angle axis
    Eigen::AngleAxisd vAl(Eigen::Quaterniond(
      lh_it->second.rotation.w,
      lh_it->second.rotation.x,
      lh_it->second.rotation.y,
      lh_it->second.rotation.z));
    // Conversion to double array
    vTl[lh_it->first] = {
      lh_it->second.transform.translation.x,
      lh_it->second.transform.translation.y,
      lh_it->second.transform.translation.z,
      vAl[0],
      vAl[1],
      vAl[2]
    };
  }

  // TODO Solver - this is temporary
  Solver * solver = new BaseSolve(calibration_.environment,
    calibration_.trackers.begin()->second)

  double * prev_pose = NULL;
  double * next_pose = NULL;
  std::string prev, next;
  hive::ViveLight horizontal_observations, vertical_observations:

  // Solution to save data
  // std::map<std::string, std::pair<hive::ViveLight, hive::ViveLight>> data;

  // Iterate tracker data
  for (auto tr_it = data_.begin(); tr_it != data_.end(); tr_it++) {
    // Iterate light data
    for (auto li_it = tr_it->second.first.begin();
      li_it != tr_it->second.first.end(); li_it++) {
      // Check if lighthouse is in calibration -- if not continue
      if (lhTv.find(li_it->lighthouse) == lhTv.end()) continue;
      // TODO Compute first individual poses
      if (li_it->axis == HORIZONTAL) {
        horizontal_observations = *li_it;
      } else {
        vertical_observations = *li_it;
      }
      // Preliminary solver
      TF tf;
      if (SolvePose(horizontal_observations,
        vertical_observations,
        tf, calibration_.trackers[tr_it->first],
        calibration.lighthouses[li_it->lighthouse], CORRECTION)) {
        double pose[6];
        prev_pose = next_pose;
        prev = next;
        next_pose = pose;
        // Horizontal cost
        if (li_it->axis == HORIZONTAL) {
          ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4> * cost =
            new ceres::DynamicAutoDiffCostFunction<PoseHorizontalCost, 4>
            (new PoseHorizontalCost(*li_it, CORRECTION));
          cost->AddParameterBlock(6);
          cost->SetNumResiduals(li_it->samples.size());
          problem.AddResidualBlock(cost, NULL, next_pose);
        // Vertical cost
        } else if (li_it->axis == VERTICAL) {
          ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4> * cost =
            new ceres::DynamicAutoDiffCostFunction<PoseVerticalCost, 4>
            (new PoseVerticalCost(*li_it, CORRECTION));
          cost->AddParameterBlock(6);
          cost->SetNumResiduals(li_it->samples.size());
          problem.AddResidualBlock(cost, NULL, next_pose);
        }
        // Smoothing cost
        if (prev_pose != NULL && next_pose != NULL) {
          ceres::CostFunction * thecost =
            new ceres::AutoDiffCostFunction<SmoothingCost, 4, 6, 6, 6, 6>
            (new SmoothingCost(SMOOTHING));
          problem.AddResidualBlock(cost, NULL,
            vTl[prev], prev_pose, vTl[next], next_pose);
        }
      }
    }
  }

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  return true;
}

int main(int argc, char ** argv)
{
  ROS_INFO("Refinery")
  return 0;
}