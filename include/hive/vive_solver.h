#ifndef HIVE_VIVE_SOLVER_H_
#define HIVE_VIVE_SOLVER_H_

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Hive imports
#include <hive/vive.h>

// ROS message imports
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>


class Solver {
public:
  virtual void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) = 0;
  virtual void ProcessLight(const hive::ViveLight::ConstPtr & msg) = 0;
  virtual bool GetTransform(geometry_msgs::TransformStamped & msg) = 0;
};

#endif // HIVE_VIVE_SOLVER_H