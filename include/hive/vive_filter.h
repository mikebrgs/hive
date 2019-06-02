#ifndef HIVE_VIVE_FILTER_H_
#define HIVE_VIVE_FILTER_H_

// ROS includes
#include <ros/ros.h>

// Hive includes
#include <hive/vive.h>
// #include <hive/vive_cost.h>
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
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>

// STD C includes
#include <math.h>

// STD C++ includes
#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <string>
#include <cmath>

#define STATE_SIZE 16         // Size of the state vector
#define NOISE_SIZE 12          // Size of the noise vector
#define LIGHT_DATA_BUFFER 4   // Size of the light data vector
#define MAHALANOBIS_MAX_DIST 3
#define IEFK_THRESHOLD 1e-5
// #define UKF_FACTOR -27.999
#define UKF_FACTOR 0.0
#define ALPHA 1e-3
#define BETA 2
#define VALID_THRESHOLD 1e-3
#define HIVE_APE_ACC 1e-4
#define MAX_OUTLIERS 1000
#define DEBUG false

namespace filter {
  // filter type
  enum type {ekf, iekf, ukf};

  typedef std::vector<hive::ViveLight> LightVector;
}

// Unscented Kalman Filter
class ViveUKF : public Solver {
public:
  ViveUKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double model_noise,
    double measure_noise,
    double factor,
    bool correction);
  ViveUKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    Eigen::MatrixXd model_noise,
    Eigen::MatrixXd measure_noise,
    double factor,
    bool correction);
  ~ViveUKF();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Print the state
  void PrintState();
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
  // Gravity estimator (in the vive frame)
  Eigen::Vector3d gravity_;
  // Last update to the solver
  ros::Time time_;
  // Covariances
  // Model covariance
  Eigen::MatrixXd model_covariance_;
  // Measurement covariance
  Eigen::MatrixXd measure_covariance_;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specs
  std::map<std::string, Lighthouse> lighthouses_;
  // Calibration environment
  Environment environment_;
  // The tracker it's solving for
  Tracker tracker_;
  // UKF factor
  double factor_;
  // If the correction parameters are to be used
  bool correction_;
  // Validity of current state
  bool valid_;
  // If the pose was already used
  bool used_;
  // Aux
  bool lastmsgwasimu_;
  // Old data for initializer
  filter::LightVector light_data_;

};

// Extended Kalman Filter
class ViveEKF : public Solver {
public:
  ViveEKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double model_noise,
    double measure_noise,
    bool correction);
  ViveEKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    Eigen::MatrixXd model_noise,
    Eigen::MatrixXd measure_noise,
    bool correction);
  ~ViveEKF();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Print the state
  void PrintState();
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
  // Gravity estimator (in the vive frame)
  Eigen::Vector3d gravity_;
  // Last update to the solver
  ros::Time time_;
  // Covatiances
  // Model covariance
  Eigen::MatrixXd model_covariance_;
  // Measurement covariance
  Eigen::MatrixXd measure_covariance_;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specs
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
  filter::LightVector light_data_;
};

// Iterated Extended Kalman Filter
class ViveIEKF : public Solver {
public:
  ViveIEKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double model_noise,
    double measure_noise,
    bool correction);
  ViveIEKF(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    Eigen::MatrixXd model_noise,
    Eigen::MatrixXd measure_noise,
    bool correction);
  ~ViveIEKF();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Print the state
  void PrintState();
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
  // Gravity estimator (in the vive frame)
  Eigen::Vector3d gravity_;
  // Last update to the solver
  ros::Time time_;
  // Covatiances
  // Model covariance
  Eigen::MatrixXd model_covariance_;
  // Measurement covariance
  Eigen::MatrixXd measure_covariance_;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specs
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
  filter::LightVector light_data_;
};


class ViveFilter : public Solver{
public:
  // Constructor
  ViveFilter();
  ViveFilter(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    double model_noise,
    double measure_noise,
    bool correction,
    filter::type ftype);
  ViveFilter(Tracker & tracker,
    std::map<std::string, Lighthouse> & lighthouses,
    Environment & environment,
    Eigen::MatrixXd model_noise,
    Eigen::MatrixXd measure_noise,
    bool correction,
    filter::type ftype);
  // Destructor
  ~ViveFilter();
  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);
  // Process a light measurement
  void ProcessLight(const hive::ViveLight::ConstPtr& msg);
  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped& msg);
  // Temporary
  void PrintState();
private: // temporary
  // EKF predict
  bool PredictEKF(const sensor_msgs::Imu & msg);
  // IEFK predict
  bool PredictIEKF(const sensor_msgs::Imu & msg);
  // UKF predict
  bool PredictUKF(const sensor_msgs::Imu & msg);
  // EKF update
  bool UpdateEKF(const hive::ViveLight & msg);
  // IEKF update
  bool UpdateIEKF(const hive::ViveLight & msg);
  // UKF update
  bool UpdateUKF(const hive::ViveLight & msg);
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
  // Gravity estimator (in the vive frame)
  Eigen::Vector3d gravity_;
  // Last update to the solver
  ros::Time time_;
  // Covatiances
  // Model covariance
  Eigen::MatrixXd model_covariance_;
  // Measurement covariance
  Eigen::MatrixXd measure_covariance_;
  // State covariance
  Eigen::MatrixXd covariance_;
  // Lighthouse specs
  std::map<std::string, Lighthouse> lighthouses_;
  // Calibration environment
  Environment environment_;
  // The tracker it's solving for
  Tracker tracker_;
  // If the correction parameters are to be used
  bool correction_;
  // Type of filter being used
  filter::type filter_type_;
  // Validity of current state
  bool valid_;
  // If the pose was already used
  bool used_;
  // Aux
  bool lastmsgwasimu_;
  // Old data for initializer
  filter::LightVector light_data_;
  // UKF stuff
  Eigen::MatrixXd ext_covariance_;
  // Outlier counter
  size_t outlier_counter;
};

#endif  // HIVE_VIVE_FILTER_H_