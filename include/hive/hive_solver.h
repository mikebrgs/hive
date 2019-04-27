#ifndef HIVE_HIVE_SOLVER_H_
#define HIVE_HIVE_SOLVER_H_

#include <ros/ros.h>

#include <hive/vive.h>
// #include <hive/vive_cost.h>
#include <hive/vive_solver.h>

// Incoming measurements
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STD C includes
#include <math.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STD C++ includes
#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <string>

typedef std::map<std::string,
  std::pair<hive::ViveLight,hive::ViveLight>> LightMap;
typedef std::map<std::string, std::pair<bool,bool>> BoolLightMap;
typedef std::map<std::string, Lighthouse> LighthouseMap;

class HiveSolver : public Solver {
 public:
  // Constructor
  HiveSolver();
  HiveSolver(Tracker & tracker,
    LighthouseMap & lighthouses,
    Environment & environment,
    bool correction);
  // Destructor
  ~HiveSolver();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped &msg);
  // Solves the pose from data
  bool Solve();

 private:
  geometry_msgs::TransformStamped pose_;
  LighthouseMap lighthouses_;
  Environment environment_;
  Tracker tracker_;
  LightMap light_data_;
  BoolLightMap light_check_;
  bool correction_;
  bool valid_;
};

#endif // HIVE_HIVE_SOLVER_H_