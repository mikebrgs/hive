#include <hive/vive_pgo.h>

InertialCost::InertialCost(sensor_msgs::Imu imu,
  geometry_msgs::TransformStamped prev_vTl,
  geometry_msgs::TransformStamped next_vTl,
  double time_step) {
  imu_ = imu;
  prev_vTl_ = prev_vTl;
  next_vTl_ = next_vTl;
  time_step_ = time_step;
  return;
}

InertialCost::~InertialCost() {
  // Nothing happens
  return;
}

template <typename T>
bool InertialCost::operator()(const T* const prev_lTt,
  const T* const next_lTt,
  T * residual) const {
  // prev_lTt's state
  Eigen::Matrix<T,3,1> prev_lPt;
  Eigen::Matrix<T,3,1> prev_lVt;
  Eigen::Matrix<T,3,3> prev_lRt;
  // next_lTt's state
  Eigen::Matrix<T,3,1> next_lPt;
  Eigen::Matrix<T,3,1> next_lVt;
  Eigen::Matrix<T,3,3> next_lRt;
  // prev_vTl's pose
  Eigen::Matrix<T,3,1> prev_vPl;
  Eigen::Matrix<T,3,3> prev_vRl;
  // prev_vTl's pose
  Eigen::Matrix<T,3,1> next_vPl;
  Eigen::Matrix<T,3,3> next_vRl;

  Eigen::Matrix<T,3,1> prev_vPt;
  Eigen::Matrix<T,3,3> prev_vRt;

  Eigen::Matrix<T,3,1> next_vPt;
  Eigen::Matrix<T,3,3> next_vRt;

  Eigen::Matrix<T,3,1> est_vPt;
  Eigen::Matrix<T,3,3> est_vRt;
  est_vPt = prev_vPt + time_step_ * prev_vPt;

  Eigen::Matrix<T,3,1> diff_vPt;
  diff_vPt = next_lPt - est_vPt;

  // Position cost
  residual[0] = diff_vPt(0);
  residual[1] = diff_vPt(0);
  residual[2] = diff_vPt(0);

  // Do all the costs
  return true;
}

PoseGraph::PoseGraph(size_t window) {
  if (window < 1) {
    std::cout << "Bad window size. Using 1." << std::endl;
    window_ = 1;
  } else {
    window_ = window;
  }
  valid_ = false;
  return;
}

PoseGraph::PoseGraph() :
  PoseGraph(10) {
    valid_ = true;
  }

PoseGraph::~PoseGraph() {
  // pass
  return;
}

void PoseGraph::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  light_data_.push_back(*msg);
  if (light_data_.size() > window_) {
    // Erase first element
    light_data_.erase(light_data_.begin());
  }
  while (light_data_.front().header.stamp > imu_data_.front().header.stamp) {
    imu_data_.erase(imu_data_.begin());
  }
  // Solve the problem
  Solve();
  return;
}

void PoseGraph::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // Save the inertial data
  imu_data_.push_back(*msg);
  return;
}

bool PoseGraph::GetTransform(geometry_msgs::TransformStamped& msg) {
  // Set the output
  msg = pose_;
  // Change the time stamp
  msg.header.stamp == ros::Time::now();
  return true;
}

bool PoseGraph::Solve() {
  return true;
}

void PoseGraph::PrintState() {
  return;
}


int main(int argc, char ** argv) {
  ROS_INFO("POSE GRAPH OPTIMIZING");
  return 0;
}