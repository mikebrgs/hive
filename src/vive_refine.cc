#include <hive/vive_refine.h>

typedef geometry_msgs::TransformStamped TF;

#define INFUNCTIONRESIDUALS 4

// TODO Cost function -- check this
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
  std::map<std::string, double[6]> lhTv;
  for (auto lh_it = calibration_.environment.lighthouses.begin();
    lh_it != calibration_.environment.lighthouses.end(); lh_it++) {
    // Conversion from quaternion to angle axis
    Eigen::AngleAxisd lhAv(Eigen::Quaterniond(
      lh_it->second.rotation.w,
      lh_it->second.rotation.x,
      lh_it->second.rotation.y,
      lh_it->second.rotation.z));
    // Conversion to double array
    lhTv[lh_it->first] = {
      lh_it->second.transform.translation.x,
      lh_it->second.transform.translation.y,
      lh_it->second.transform.translation.z,
      lhAv[0],
      lhAv[1],
      lhAv[2]
    };
  }

  // TODO Solver - this is temporary
  Solver * solver = new BaseSolve(calibration_.environment,
    calibration_.trackers.begin()->second)

  // Iterate tracker data
  for (auto tr_it = data_.begin(); tr_it != data_.end(); tr_it++) {
    // Iterate light data
    for (auto li_it = tr_it->second.first.begin();
      li_it != tr_it->second.first.end(); li_it++) {
      // Check if lighthouse is in calibration
      if (lhTv.find(li_it->lighthouse) == lhTv.end()) continue;
      // TODO Compute first individual poses

      // Do cost function
      // ceres::CostFunction * thecost =
      //   new ceres::AutoDiffCostFunction<RefineCostFunctor, INFUNCTIONRESIDUALS, 6, 6>
      //   (new RefineCostFunctor(pose1, pose2));
      // problem.AddResidualBlock(thecost, NULL, lh1, lh2);
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