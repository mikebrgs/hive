#include <hive/vive_pgo.h>

InertialCost::InertialCost(sensor_msgs::Imu imu,
  hive::ViveLight prev,
  hive::ViveLight next) {
  imu_ = imu;
  prev_ = prev;
  next_ = next;
  return;
}

InertialCost::~InertialCost() {
  // Nothing happens
  return;
}

template <typename T>
bool InertialCost::operator()(const T* const prev_lTt,
  const T* const prev_vTl,
  const T* const next_lTt,
  const T* const next_vTl,
  T * residual) const {
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