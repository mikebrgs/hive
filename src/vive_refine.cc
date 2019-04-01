#include <hive/vive_refine.h>

// Internal datatypes
struct Sweep {
  LightVec lights;
  std::string lighthouse;
  uint8_t axis;
};
typedef std::vector<Sweep> SweepVec;
typedef std::pair<SweepVec, ImuVec> DataPair;         // pair of Light data and Imu data - change imu
typedef std::map<std::string, DataPair> DataPairMap;         // map of trackers


// Cost function

Refinery::Refinery(Calibration & calibration) {
  calibration_ = calibration;
  return;
}

Refinery::~Refinery() {
  // pass
}

bool AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // pass
}

bool AddLight(const hive::ViveLight::ConstPtr& msg) {
  // Save the data
}

bool Solve() {
  // Solve the problem

}

int main(int argc, char ** argv)
{
  ROS_INFO("Refinery")
  return 0;
}