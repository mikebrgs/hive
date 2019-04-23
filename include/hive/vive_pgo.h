#ifndef HIVE_VIVE_PGO
#define HIVE_VIVE_PGO

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive_general.h>
#include <hive/vive_solve.h>
#include <hive/vive.h>

// Hive msgs
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <hive/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

// Eigen C++ includes
#include <vector>
#include <map>
#include <string>


class PoseGraph : public Solver {
public:
  // Constructor
  PoseGraph();
  PoseGraph(Environment environment,
  Tracker tracker,
  std::map<std::string, Lighthouse> lighthouses,
  size_t window,
  bool correction);
  // Destructor
  ~PoseGraph();
  // New light data
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Aux method
  void RemoveLight();
  // New Imu data
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Aux method
  void RemoveImu();
  // Get the tracker's pose
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // 
  void AddPoseBack();
  void AddPoseFront();
  void ApplyLimits();
  // Solve the problem
  bool Solve();
  // Prinst stuff
  void PrintState();
  
private:
  // The pose
  geometry_msgs::TransformStamped pose_;
  // Light data
  std::vector<hive::ViveLight> light_data_;
  // Inertial data
  std::vector<sensor_msgs::Imu> imu_data_;
  // Calibrated environment
  Environment environment_;
  // Tracker
  Tracker tracker_;
  // Lighthouses specs
  std::map<std::string, Lighthouse> lighthouses_;
  // Correction
  bool correction_;
  // Optimization window
  size_t window_;
  // Validity
  bool valid_;
  // Internal
  std::vector<double*> poses_;
  bool lastposewasimu_;
};

// Light cost - Cost using the poses in the imu frame
class ViveHorizontalCost
{
public:
  // Constructor
  ViveHorizontalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv, // vive to lighthouse
    geometry_msgs::Transform tTi, // IMU to light
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveHorizontalCost();
  // Ceres operator
  template <typename T> bool operator()(const T* const * parameters,
    T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lTv_, tTi_;
};

class ViveVerticalCost
{
public:
  // Constructor
  ViveVerticalCost(hive::ViveLight data,
    geometry_msgs::Transform lTv, // Vive to lighthouse
    geometry_msgs::Transform tTi, // IMU to light
    Tracker tracker,
    Motor lighthouse,
    bool correction);
  // Destructor
  ~ViveVerticalCost();
  // Ceres operator
  template <typename T> bool operator()(const T* const * parameters,
    T* residual) const;
private:
  bool correction_;
  Tracker tracker_;
  Motor lighthouse_;
  hive::ViveLight data_;
  geometry_msgs::Transform lTv_, tTi_;
};

// Inertial cost function
class InertialCost {
public:
  InertialCost(sensor_msgs::Imu imu,
    geometry_msgs::Vector3 gravity,
    double time_step,
    double trust_weight);
  ~InertialCost();
  template <typename T> bool operator()(const T* const prev_vTi,
    const T* const next_vTi,
    T * residual) const;
private:
  // Inertial data
  sensor_msgs::Imu imu_;
  // Light data
  hive::ViveLight prev_, next_;
  // Gravity
  geometry_msgs::Vector3 gravity_;
  // Time step and weight
  double time_step_, trust_weight_;
};

#endif // HIVE_VIVE_PGO