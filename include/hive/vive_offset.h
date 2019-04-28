/*
 * @version 1.0
 * @author Miguel Rego Borges
 * Instituto Superior Tecnico - University of Lisbon
 * Purpose: calculate offset between vive's frame and optitrack's
*/

#ifndef HIVE_VIVE_OFFSET_H_
#define HIVE_VIVE_OFFSET_H_

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Standard C includes
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Standard C++ includes
#include <iostream>
#include <string>

// Hive includes
#include <hive/vive.h>
#include <hive/vive_cost.h>
#include <hive/vive_solve.h>
#include <hive/vive_general.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Visp
#include <visp3/vision/vpHandEyeCalibration.h>

// ROS msgs
#include <hive/ViveLight.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>



#define DISTANCE_THRESH 0.1
#define ANGLE_THRESH 0.17
#define TIME_THRESH 0.01
#define CAUCHY 0.05
#define CERES_ITERATIONS 500

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;

class HiveOffset
{
public:
  // Constructor - steps indicates if it averages sets of poses
  HiveOffset(double angle_factor, bool refine, bool steps);
  // Destructor
  ~HiveOffset();
  // Add a pose computed from Vive
  bool AddVivePose(const TF& pose);
  // Add a pose computed from OptiTrack
  bool AddOptiTrackPose(const TF& pose);
  // Change the pose (in the case of averaging)
  bool NextPose();
  // Get the offsets
  bool GetOffset(TFs & offsets);
  // Estimate the offset with Hand Eye Calibration algorithm
  static TFs VispEstimateOffset(TFs optitrack, TFs vive);
  static bool CeresEstimateOffset(TFs& optitrack, TFs& vive, TFs& offsets);
  // Refine with ceres the estimate of the offset
  static TFs RefineOffset(TFs optitrack, TFs vive, TFs offset);

private:
  // Data
  TFs tmp_vive_, vive_;
  TFs tmp_optitrack_, optitrack_;
  // Parameters
  bool steps_;
  bool refine_;
  double angle_factor_;
};


#endif // HIVE_VIVE_OFFSET_H_