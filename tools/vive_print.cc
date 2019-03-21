// This package code
#include <hive/vive_general.h>
#include <hive/vive.h>
#include <hive/vive_solve.h>
#include <hive/vive_calibrate.h>
#include <hive/vive_visualization.h>

// Standard C includes
#include <stdio.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_broadcaster.h>

// Standard C++ includes
#include <iostream>

// Services
#include <hive/ViveConfig.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <hive/ViveLight.h>
#include <hive/ViveCalibration.h>
#include <hive/ViveCalibrationGeneral.h>
#include <hive/ViveCalibrationTrackerArray.h>
#include <hive/ViveCalibrationLighthouseArray.h>

uint32_t thesensor = 6;

void LightCallback(const hive::ViveLight::ConstPtr& msg) {
  // Check the current state of the system
  std::cout << msg->lighthouse << std::endl;
  return;
}

int main(int argc, char ** argv)
{
  ros::Subscriber sub_light;

  ros::init(argc, argv, "printer");
  ros::NodeHandle nh;

  sub_light = nh.subscribe(TOPIC_HIVE_LIGHT, 1000,
    &LightCallback);

  ros::spin();

  return 0;
}