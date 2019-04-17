#ifndef HIVE_VIVE_OPTIMIZER_H_
#define HIVE_VIVE_OPTIMIZER_H_

// ROS includes
#include <ros/ros.h>

// ROS message imports
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>


class Optimizer {
public:
  virtual bool AddImu(const sensor_msgs::Imu::ConstPtr& msg) = 0;
  virtual bool AddLight(const hive::ViveLight::ConstPtr & msg) = 0;
  virtual bool Solve() = 0;
};

#endif // HIVE_VIVE_OPTIMIZER_H