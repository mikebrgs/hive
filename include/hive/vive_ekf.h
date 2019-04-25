#ifndef HIVE_VIVE_EKF_H_
#define HIVE_VIVE_EKF_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive.h>
#include <hive/vive_cost.h>
#include <hive/vive_solver.h>
#include <hive/vive_general.h>

// Incoming measurements
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STD C includes
#include <math.h>

// STD C++ includes
#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <string>

#define STATE_SIZE 13         // Size of the state vector
#define NOISE_SIZE 9          // Size of the noise vector
#define LIGHT_DATA_BUFFER 4   // Size of the light data vector

typedef std::map<std::string,
  std::pair<hive::ViveLight*,hive::ViveLight*>> LightMap;

typedef std::vector<hive::ViveLight> LightVector;


class ViveEKF : public Solver{
public:
  // Constructor
  ViveEKF();
  ViveEKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double model_noise,
    double measure_noise,
    bool correction);
  // Destructor
  ~ViveEKF();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Temporary
  void PrintState();
private: // temporary
  // Internal method
  bool Predict(const sensor_msgs::Imu & msg);
  // Internal method
  bool Update(const hive::ViveLight & msg);
  // Validity
  bool Valid();
  // Initialize estimates
  bool Initialize();
private:
  // State
  // Positon of the imu frame in the Vive frame
  Eigen::Vector3d position_;
  // Velocity of the imu frame in the Vive frame
  Eigen::Vector3d velocity_;
  // Orientation of the imu frame in the Vive frame
  Eigen::Quaterniond rotation_;
  // Bias of the imu frame in the Vive frame
  Eigen::Vector3d bias_;
  // Last update to the solver
  ros::Time time_;
  // Covatiances
  // Model covariance
  Eigen::MatrixXd model_covariance_;
  // Measurement covariance
  Eigen::MatrixXd measure_covariance_;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specifications
  std::map<std::string, Lighthouse> lighthouses_;
  // Calibration environment
  Environment environment_;
  // The tracker it's solving for
  Tracker tracker_;
  // If the correction parameters are to be used
  bool correction_;
  // Validity of current state
  bool valid_;
  // If the pose was already used
  bool used_;
  // Aux
  bool lastmsgwasimu_;
  // Old data for initializer
  LightVector light_data_;
};

#endif  // HIVE_VIVE_EKF_H_