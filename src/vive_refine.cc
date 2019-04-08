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
  template <typename T> bool operator()(const T* const prevPose,
    const T* const prevT,
    const T* const nextPose,
    const T* const nextT,
    T * residual) const {
    // cost function here

    // Still missing the frame conversion here

    // Translation cost with smoothing
    residual[0] = smoothing_ * (prevPose[0] - nextPose[0]);
    residual[1] = smoothing_ * (prevPose[1] - nextPose[1]);
    residual[2] = smoothing_ * (prevPose[2] - nextPose[2]);
    // Rotation cost with smoothing
    Eigen::AngleAxis<T> prevA(prevPose[3], prevPose[4], prevPose[5]);
    Eigen::AngleAxis<T> nextA(nextPose[3], nextPose[4], nextPose[5]);
    residual[3] = smoothing_ * (nextA.inverse() * prevA).angle();
    return true;
  }
private:
  smoothing_;
};

class PoseCost
{
public:
  PoseCost(/*Data here*/) {
    // Save data here
  }

  template <typename T> bool operator()(const T* const parameters,
    T * residual) const {
    // Do cost here
    return true;
  }
private:
  // Save stuff here
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