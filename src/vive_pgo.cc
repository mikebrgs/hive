#include <hive/vive_pgo.h>

#define TRUST 0.4

enum DataType {imu, light};

namespace pgo {
  // Light cost - Cost using the poses from the imu frame to the vive frame
  class ViveHorizontalCost
  {
  public:
    // Constructor
    ViveHorizontalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl, // vive to lighthouse
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveHorizontalCost();
    // Ceres operator
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
    geometry_msgs::Transform vTl_;
  };

  // Light cost - Cost using the poses from the imu frame to the vive frame
  class ViveVerticalCost
  {
  public:
    // Constructor
    ViveVerticalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl, // Vive to lighthouse
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveVerticalCost();
    // Ceres operator
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
    geometry_msgs::Transform vTl_;
  };

  // Inertial cost function
  class InertialCost {
  public:
    InertialCost(sensor_msgs::Imu imu,
      geometry_msgs::Vector3 gravity,
      double time_step,
      double trust_weight,
      bool verbose = false);
    ~InertialCost();
    template <typename T> bool operator()(const T* const prev_vTi,
      const T* const next_vTi,
      const T* const acc_bias,
      const T* const ang_bias,
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
    // Other
    bool verbose_;
  };

  ViveHorizontalCost::ViveHorizontalCost(hive::ViveLight data,
      geometry_msgs::Transform vTl,
      Tracker tracker,
      Motor lighthouse,
      bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    vTl_ = vTl;
    return;
  }

  ViveHorizontalCost::~ViveHorizontalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveHorizontalCost::operator()(const T* const * parameters,
    T* residual) const {
    // Optimization parameters
    Eigen::Matrix<T, 3, 1> vPi;
    vPi << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRi;
    ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

    // Lighthouse pose
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << T(vTl_.translation.x),
      T(vTl_.translation.y),
      T(vTl_.translation.z);
    Eigen::Quaternion<T> vQl(
      T(vTl_.rotation.w),
      T(vTl_.rotation.x),
      T(vTl_.rotation.y),
      T(vTl_.rotation.z));
    Eigen::Matrix<T, 3, 3> vRl;
    vRl = vQl.toRotationMatrix();

    // Inertial transform
    Eigen::Matrix<T,3,1> tPi;
    tPi << T(tracker_.imu_transform.translation.x),
      T(tracker_.imu_transform.translation.y),
      T(tracker_.imu_transform.translation.z);
    Eigen::Quaternion<T> tQi(
      T(tracker_.imu_transform.rotation.w),
      T(tracker_.imu_transform.rotation.x),
      T(tracker_.imu_transform.rotation.y),
      T(tracker_.imu_transform.rotation.z));
    Eigen::Matrix<T, 3, 3> tRi;
    tRi = tQi.toRotationMatrix();

    // Invert inertial transform
    Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
    Eigen::Matrix<T,3,3> iRt = tRi.transpose();

    // Vive and IMU frame to lighthouse and tracker
    Eigen::Matrix<T, 3, 1> lPv;
    lPv = - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRv;
    lRv = vRl.transpose();
    Eigen::Matrix<T, 3, 1> lPt;
    lPt = lRv * (vRi * iPt + vPi) + lPv;
    Eigen::Matrix<T, 3, 3> lRt;
    lRt = lRv * vRi * iRt;

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
      } else {
        ang = atan(x);
      }

      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }

  ViveVerticalCost::ViveVerticalCost(hive::ViveLight data,
    geometry_msgs::Transform vTl,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    vTl_ = vTl;
    return;
  }

  ViveVerticalCost::~ViveVerticalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveVerticalCost::operator()(const T* const * parameters,
    T* residual) const {
    // Optimization parameters
    Eigen::Matrix<T, 3, 1> vPi;
    vPi << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRi;
    ceres::AngleAxisToRotationMatrix(&parameters[0][6], vRi.data());

    // Lighthouse pose
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << T(vTl_.translation.x),
      T(vTl_.translation.y),
      T(vTl_.translation.z);
    Eigen::Quaternion<T> vQl(
      T(vTl_.rotation.w),
      T(vTl_.rotation.x),
      T(vTl_.rotation.y),
      T(vTl_.rotation.z));
    Eigen::Matrix<T, 3, 3> vRl;
    vRl = vQl.toRotationMatrix();

    // Inertial transform
    Eigen::Matrix<T,3,1> tPi;
    tPi << T(tracker_.imu_transform.translation.x),
      T(tracker_.imu_transform.translation.y),
      T(tracker_.imu_transform.translation.z);
    Eigen::Quaternion<T> tQi(
      T(tracker_.imu_transform.rotation.w),
      T(tracker_.imu_transform.rotation.x),
      T(tracker_.imu_transform.rotation.y),
      T(tracker_.imu_transform.rotation.z));
    Eigen::Matrix<T, 3, 3> tRi;
    tRi = tQi.toRotationMatrix();

    // Invert inertial transform
    Eigen::Matrix<T,3,1> iPt = - tRi.transpose() * tPi;
    Eigen::Matrix<T,3,3> iRt = tRi.transpose();

    // Vive and IMU frame to lighthouse and tracker
    Eigen::Matrix<T, 3, 1> lPv;
    lPv = - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRv;
    lRv = vRl.transpose();
    Eigen::Matrix<T, 3, 1> lPt;
    lPt = lRv * (vRi * iPt + vPi) + lPv;
    Eigen::Matrix<T, 3, 3> lRt;
    lRt = lRv * vRi * iRt;

    size_t counter = 0;
    for (auto li_it = data_.samples.begin();
      li_it != data_.samples.end(); li_it++) {
      auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
      if (sensor_it == tracker_.sensors.end()) return false;
      Eigen::Matrix<T, 3, 1> tPs;
      tPs << T(sensor_it->second.position.x),
        T(sensor_it->second.position.y),
        T(sensor_it->second.position.z);

      Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

      T ang; // The final angle
      T x = (lPs(0)/lPs(2)); // Horizontal angle
      T y = (lPs(1)/lPs(2)); // Vertical angle
      T phase = T(lighthouse_.phase);
      T tilt = T(lighthouse_.tilt);
      T gib_phase = T(lighthouse_.gib_phase);
      T gib_mag = T(lighthouse_.gib_magnitude);
      T curve = T(lighthouse_.curve);

      if (correction_) {
        ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
      } else {
        ang = atan(y);
      }


      residual[counter] = T(li_it->angle) - ang;
      counter++;
    }
    return true;
  }

  InertialCost::InertialCost(sensor_msgs::Imu imu,
    geometry_msgs::Vector3 gravity,
    // geometry_msgs::Vector3 gyr_bias,
    double time_step,
    double trust_weight,
    bool verbose) {
    imu_ = imu;
    gravity_ = gravity;
    time_step_ = time_step;
    trust_weight_ = trust_weight;
    // gyr_bias_ = gyr_bias;
    verbose_ = verbose;
    return;
  }

  InertialCost::~InertialCost() {
    // Nothing happens
    return;
  }

  template <typename T>
  bool InertialCost::operator()(const T* const prev_vTi,
    const T* const next_vTi,
    const T* const bias_acc,
    const T* const bias_ang,
    T * residual) const {
    // Biases
    Eigen::Matrix<T,3,1> b_iA;
    b_iA << T(bias_acc[0]),
      T(bias_acc[1]),
      T(bias_acc[2]);
    Eigen::Matrix<T,3,1> b_iW;
    b_iW << T(bias_ang[0]),
      T(bias_ang[1]),
      T(bias_ang[2]);

    // prev_vTt's state
    // Position
    Eigen::Matrix<T,3,1> prev_vPi;
    prev_vPi << prev_vTi[0],
      prev_vTi[1],
      prev_vTi[2];
    // Velocity
    Eigen::Matrix<T,3,1> prev_vVi;
    prev_vVi << prev_vTi[3],
      prev_vTi[4],
      prev_vTi[5];
    // Angle axis in vector
    Eigen::Matrix<T,3,1> prev_vAi;
    prev_vAi << prev_vTi[6],
      prev_vTi[7],
      prev_vTi[8];
    // Rotation matrix
    Eigen::Matrix<T,3,3> prev_vRi;// = prev_vQi.toRotationMatrix();
    ceres::AngleAxisToRotationMatrix(&prev_vTi[6], prev_vRi.data());
    // Angle axis eigen structure
    // Eigen::AngleAxis<T> prev_vAAi(prev_vRi);

    // Eigen Quaternion
    Eigen::Quaternion<T> prev_vQi(prev_vRi);
    // std::cout << prev_vAAi.axis()(0) << ", "
    //   << prev_vAAi.axis()(1) << ", "
    //   << prev_vAAi.axis()(2) << ", "
    //   << prev_vAAi.angle() << std::endl;

    // next_vTt's state
    // Position
    Eigen::Matrix<T,3,1> next_vPi;
    next_vPi << next_vTi[0],
      next_vTi[1],
      next_vTi[2];
    // Velocity
    Eigen::Matrix<T,3,1> next_vVi;
    next_vVi << next_vTi[3],
      next_vTi[4],
      next_vTi[5];
    // Angle axis in vector
    Eigen::Matrix<T,3,1> next_vAi;
    next_vAi << next_vTi[6],
      next_vTi[7],
      next_vTi[8];
    // Rotation matrix
    Eigen::Matrix<T,3,3> next_vRi;// = next_vQi.toRotationMatrix();
    ceres::AngleAxisToRotationMatrix(&next_vTi[6], next_vRi.data());
    // Angle axis eigen structure
    // Eigen::AngleAxis<T> next_vAAi(next_vAi.norm(),
    //   next_vAi.normalized());
    // Eigen Quaternion
    Eigen::Quaternion<T> next_vQi(next_vRi);

    // Inertial measurements
    Eigen::Matrix<T,3,1> iW;
    iW << T(imu_.angular_velocity.x),
      T(imu_.angular_velocity.y),
      T(imu_.angular_velocity.z);
    Eigen::Matrix<T,3,1> iA;
    iA << T(imu_.linear_acceleration.x),
      T(imu_.linear_acceleration.y),
      T(imu_.linear_acceleration.z);

    // Inertial bias
    // Eigen::Matrix<T,3,1> iB;
    // iB << T(gyr_bias_.x),
    //   T(gyr_bias_.y),
    //   T(gyr_bias_.z);

    // Gravity
    Eigen::Matrix<T,3,1> vG;
    vG << T(gravity_.x),
      T(gravity_.y),
      T(gravity_.z);

    // Inertial predictions
    // Position prediction
    Eigen::Matrix<T,3,1> est_vPi;
    est_vPi = prev_vPi + T(time_step_) * prev_vVi + T(0.5 * time_step_ * time_step_) * (vG - prev_vRi * iA);
    // Velocity prediction
    Eigen::Matrix<T,3,1> est_vVi;
    est_vVi = prev_vVi + T(time_step_) * (vG - (prev_vRi * (iA - b_iA)));

    // std::cout << (prev_vRi * iA)(0) << ", " << (prev_vRi * iA)(1) << ", " << (prev_vRi * iA)(2) << std::endl;
    // Quaternion derivative matrix
    Eigen::Matrix<T,4,3> Omega;
    Omega << -prev_vQi.x(), -prev_vQi.y(), -prev_vQi.z(),
      prev_vQi.w(), -prev_vQi.z(), prev_vQi.y(),
      prev_vQi.z(), prev_vQi.w(), -prev_vQi.x(),
      -prev_vQi.y(), prev_vQi.x(), prev_vQi.w();
    // Temporary previous orientation in vector
    Eigen::Matrix<T,4,1> trev_vQi; // temporary previous
    trev_vQi << prev_vQi.w(),
      prev_vQi.x(),
      prev_vQi.y(),
      prev_vQi.z();
    Eigen::Matrix<T,4,1> text_vQi; // temporary next
    // Temporary next orientation in vector
    text_vQi = trev_vQi + T(time_step_) * T(0.5) * Omega * (iW - b_iW);
    // std::cout << "trev_vQi " << trev_vQi(0) << ", "
    //   << trev_vQi(1) << ", "
    //   << trev_vQi(2) << ", "
    //   << trev_vQi(3) << std::endl;
    // std::cout << "iW " << iW(0) << ", "
    //   << iW(1) << ", "
    //   << iW(2) << std::endl;
    // std::cout << "iB " << iB(0) << ", "
    //   << iB(1) << ", "
    //   << iB(2) << std::endl;
    // text_vQi.normalize(); // maybe remove
    // Fix small errors
    Eigen::Quaternion<T> est_vQi(text_vQi(0),
      text_vQi(1),
      text_vQi(2),
      text_vQi(3));
    Eigen::Matrix<T,3,3> est_vRi = est_vQi.normalized().toRotationMatrix();

    // Estimated from inertia vs next
    Eigen::Matrix<T,3,1> dP;
    Eigen::Matrix<T,3,1> dV;
    Eigen::Matrix<T,3,3> dR;
    dP = next_vPi - est_vPi;
    dV = next_vVi - est_vVi;
    dR = next_vRi.transpose() * est_vRi;
    // std::cout << est_vRi(0,0) << ", " << est_vRi(0,1) << ", " << est_vRi(0,2) << ", ";
    // std::cout << est_vRi(1,0) << ", " << est_vRi(1,1) << ", " << est_vRi(1,2) << ", ";
    // std::cout << est_vRi(2,0) << ", " << est_vRi(2,1) << ", " << est_vRi(2,2) << std::endl;

    // Position cost
    residual[0] = T(sqrt(trust_weight_)) * dP(0);
    residual[1] = T(sqrt(trust_weight_)) * dP(1);
    residual[2] = T(sqrt(trust_weight_)) * dP(2);
    // Velocity cost
    // residual[3] = prev_vPi(0) - next_vPi(0);
    // residual[4] = prev_vPi(1) - next_vPi(1);
    // residual[5] = prev_vPi(2) - next_vPi(2);
    residual[3] = T(sqrt(trust_weight_)) * dV(0);
    residual[4] = T(sqrt(trust_weight_)) * dV(1);
    residual[5] = T(sqrt(trust_weight_)) * dV(2);
    // Orientation cost
    T aa[3];
    ceres::RotationMatrixToAngleAxis(dR.data(), aa);
    // std::cout << aa[0] << ", " << aa[1] << ", " << aa[2] << std::endl;
    residual[6] = T(sqrt(trust_weight_)) *
      sqrt(0.001 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); // WATCH OUT FOR THIS
      // Watch out for bias
    return true;
  }

  // How close the poses should be to each other
  class ClosenessCost {
  public:
    // Constructor to pass data
    // A good smoothing factor is 0.1, but it may be changed
    explicit ClosenessCost(double smoothing,
      double vel_smooth);

    // Function for ceres solver with parameters (different frames)
    template <typename T> bool operator()(const T* const prev_vTt,
      const T* const next_vTt,
      T * residual) const;

  private:
    double smoothing_;
    double vel_smooth_;
  };

  // Constructor to pass data
  ClosenessCost::ClosenessCost(double smoothing,
    double vel_smooth) {
    smoothing_ = smoothing;
    vel_smooth_ = vel_smooth;
  }
  // Function for ceres solver with parameters
  template <typename T> bool ClosenessCost::operator()(const T* const prev_vTt,
    const T* const next_vTt,
    T * residual) const {
    // cost function here

    // Frame conversion
    Eigen::Matrix<T, 3, 1> prev_vPt;
    prev_vPt << prev_vTt[0],
      prev_vTt[1],
      prev_vTt[2];
    Eigen::Matrix<T, 3, 1> next_vPt;
    next_vPt << next_vTt[0],
      next_vTt[1],
      next_vTt[2];
    Eigen::Matrix<T, 3, 1> prev_vVt;
    prev_vVt << prev_vTt[3],
      prev_vTt[4],
      prev_vTt[5];
    Eigen::Matrix<T, 3, 1> next_vVt;
    next_vVt << next_vTt[3],
      next_vTt[4],
      next_vTt[5];
    Eigen::Matrix<T, 3, 3> prev_vRt;
    ceres::AngleAxisToRotationMatrix(&prev_vTt[6], prev_vRt.data());
    Eigen::Matrix<T, 3, 3> next_vRt;
    ceres::AngleAxisToRotationMatrix(&next_vTt[6], next_vRt.data());

    // // Translation cost with smoothing
    residual[0] = T(sqrt(smoothing_)) * (prev_vPt(0) - next_vPt(0));
    residual[1] = T(sqrt(smoothing_)) * (prev_vPt(1) - next_vPt(1));
    residual[2] = T(sqrt(smoothing_)) * (prev_vPt(2) - next_vPt(2));
    residual[3] = T(sqrt(1.0e0 * vel_smooth_*smoothing_)) * (prev_vVt(0) - next_vVt(0));
    residual[4] = T(sqrt(1.0e0 * vel_smooth_*smoothing_)) * (prev_vVt(1) - next_vVt(1));
    residual[5] = T(sqrt(1.0e0 * vel_smooth_*smoothing_)) * (prev_vVt(2) - next_vVt(2));
    // // Rotation cost with smoothing
    T aa[3];
    Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
    ceres::RotationMatrixToAngleAxis(R.data(), aa);
    residual[6] = T(1.0e0 * ROTATION_FACTOR) * T(sqrt(smoothing_))*
      sqrt(0.001 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
    return true;
  }

};

PoseGraph::PoseGraph(Environment environment,
  Tracker tracker,
  std::map<std::string, Lighthouse> lighthouses,
  size_t window,
  double trust,
  double first_factor,
  bool correction) {
  if (window < 2) {
    std::cout << "Bad window size. Using 2." << std::endl;
    window_ = 2;
  } else {
    window_ = window;
  }
  valid_ = false;
  lastposewasimu_ = false;
  correction_ = correction;
  trust_ = trust;
  tracker_ = tracker;
  environment_ = environment;
  lighthouses_ = lighthouses;
  // force_first_ = force_first;
  first_factor_ = first_factor;
  return;
}

PoseGraph::PoseGraph() {
  valid_ = true;
  return;
}

PoseGraph::~PoseGraph() {
  // pass
  return;
}


bool PoseGraph::Valid() {
  // Parameters to validate pose
  double cost = 0;
  double sample_counter = 0;
  // Pose of the tracker in the vive frame
  Eigen::Vector3d vPt(pose_.transform.translation.x,
    pose_.transform.translation.y,
    pose_.transform.translation.z);
  Eigen::Quaterniond vQt(pose_.transform.rotation.w,
    pose_.transform.rotation.x,
    pose_.transform.rotation.y,
    pose_.transform.rotation.z);
  Eigen::Matrix3d vRt = vQt.toRotationMatrix();

  // std::cout << "Val T: "
  //   << vPt(0) << ", "
  //   << vPt(1) << ", "
  //   << vPt(2) << ", "
  //   << vQt.w() << ", "
  //   << vQt.x() << ", "
  //   << vQt.y() << ", "
  //   << vQt.z() << std::endl;

  for (auto light_sample : light_data_) {
    // Lighthouse in the vive frame
    Eigen::Vector3d vPl(
      environment_.lighthouses[light_sample.lighthouse].translation.x,
      environment_.lighthouses[light_sample.lighthouse].translation.y,
      environment_.lighthouses[light_sample.lighthouse].translation.z);
    Eigen::Quaterniond vQl(
      environment_.lighthouses[light_sample.lighthouse].rotation.w,
      environment_.lighthouses[light_sample.lighthouse].rotation.x,
      environment_.lighthouses[light_sample.lighthouse].rotation.y,
      environment_.lighthouses[light_sample.lighthouse].rotation.z);
    Eigen::Matrix3d vRl = vQl.toRotationMatrix();

    // std::cout << "Val L: "
    //   << vPl(0) << ", "
    //   << vPl(1) << ", "
    //   << vPl(2) << ", "
    //   << vQl.w() << ", "
    //   << vQl.x() << ", "
    //   << vQl.y() << ", "
    //   << vQl.z() << std::endl;

    // Convert pose to lighthouse frame
    Eigen::Vector3d lPt = vRl.transpose() * (vPt) + ( - vRl.transpose() * vPl);
    Eigen::Matrix3d lRt = vRl.transpose() * vRt;


    // std::cout << "Val lT: "
    //   << lPt(0) << ", "
    //   << lPt(1) << ", "
    //   << lPt(2) << ", "
    //   << Eigen::Quaterniond(lRt).w() << ", "
    //   << Eigen::Quaterniond(lRt).x() << ", "
    //   << Eigen::Quaterniond(lRt).y() << ", "
    //   << Eigen::Quaterniond(lRt).z() << std::endl;
    for (auto sample : light_sample.samples) {
      Eigen::Vector3d tPs(tracker_.sensors[sample.sensor].position.x,
        tracker_.sensors[sample.sensor].position.y,
        tracker_.sensors[sample.sensor].position.z);
      // Sensor in lighthouse frame
      Eigen::Vector3d lPs = lRt * tPs + lPt;
      // Horizontal Angle
      if (light_sample.axis == HORIZONTAL) {
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_sample.lighthouse].horizontal_motor.phase;
        double tilt = lighthouses_[light_sample.lighthouse].horizontal_motor.tilt;
        double gib_phase = lighthouses_[light_sample.lighthouse].horizontal_motor.gib_phase;
        double gib_mag = lighthouses_[light_sample.lighthouse].horizontal_motor.gib_magnitude;
        double curve = lighthouses_[light_sample.lighthouse].horizontal_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
        } else {
          ang = atan(x);
        }
        // Adding to cost
        cost += pow(sample.angle - ang,2);
        sample_counter++;
      // Vertical Angle
      } else if (light_sample.axis == VERTICAL) {
        double ang;
        double x = (lPs(0)/lPs(2)); // Horizontal angle
        double y = (lPs(1)/lPs(2)); // Vertical angle
        double phase = lighthouses_[light_sample.lighthouse].vertical_motor.phase;
        double tilt = lighthouses_[light_sample.lighthouse].vertical_motor.tilt;
        double gib_phase = lighthouses_[light_sample.lighthouse].vertical_motor.gib_phase;
        double gib_mag = lighthouses_[light_sample.lighthouse].vertical_motor.gib_magnitude;
        double curve = lighthouses_[light_sample.lighthouse].vertical_motor.curve;
        // Correction
        if (correction_) {
          ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
        } else {
          ang = atan(y);
        }
        // Adding to cost
        cost += pow(sample.angle - ang,2);
        sample_counter++;
      }
    }
  }

  // std::cout << "COST: " << cost << std::endl;

  if (cost > 1e-4 * sample_counter)
    return false;

  return true;
}

void PoseGraph::ProcessLight(const hive::ViveLight::ConstPtr& msg) {
  // Create copy to clean
  hive::ViveLight * clone_msg = new hive::ViveLight(*msg);
  // Erase outliers
  auto sample_it = clone_msg->samples.begin();
    while (sample_it != clone_msg->samples.end()) {
    if (sample_it->angle > -M_PI/3.0 && sample_it->angle < M_PI / 3.0) {
      sample_it++;
    } else {
      clone_msg->samples.erase(sample_it);
    }
  }
  if (clone_msg->samples.size() == 0) {
    return;
  }
  // Save the copy
  light_data_.push_back(*clone_msg);

  while (light_data_.size() > window_) {
    light_data_.erase(light_data_.begin());
    if (poses_.size() > 0) {
      poses_.erase(poses_.begin());
    }
    size_t imu_counter = 0;
    while (imu_data_.size() > 0 
      && imu_data_.front().header.stamp < light_data_.front().header.stamp) {
      imu_counter++;
      imu_data_.erase(imu_data_.begin());
      if (poses_.size() > 0 && imu_counter > 1) {
        poses_.erase(poses_.begin());
      }
    }
  }

  // Solve the problem
  if (!Solve()) {
    light_data_.pop_back();
  }

  valid_ = Valid();

  return;
}

void PoseGraph::ProcessImu(const sensor_msgs::Imu::ConstPtr& msg) {
  // Create copy to clean
  sensor_msgs::Imu * clone_msg = new sensor_msgs::Imu(*msg);
  // Save the copy
  imu_data_.push_back(*clone_msg);

  return;
}

// void PoseGraph::RemoveImu() {
//   imu_data_.erase(imu_data_.begin());
//   poses_.erase(poses_.begin());
//   return;
// }

// void PoseGraph::AddPoseBack() {
//   double * new_pose = new double[9];
//   if (poses_.size() <= 0) {
//     new_pose[0] = 0.0;
//     new_pose[1] = 0.0;
//     new_pose[2] = 1.0;
//     new_pose[3] = 0.0;
//     new_pose[4] = 0.0;
//     new_pose[5] = 0.0;
//     new_pose[6] = 0.0;
//     new_pose[7] = 0.0;
//     new_pose[8] = 0.0;
//   } else {
//     new_pose[0] = poses_.back()[0];
//     new_pose[1] = poses_.back()[1];
//     new_pose[2] = poses_.back()[2];
//     new_pose[3] = poses_.back()[3];
//     new_pose[4] = poses_.back()[4];
//     new_pose[5] = poses_.back()[5];
//     new_pose[6] = poses_.back()[6];
//     new_pose[7] = poses_.back()[7];
//     new_pose[8] = poses_.back()[8];
//   }
//   poses_.push_back(new_pose);
//   return;
// }

// void PoseGraph::AddPoseFront() {
//   double * new_pose = new double[9];
//   if (poses_.size() <= 0) {
//     new_pose[0] = 0.0;
//     new_pose[1] = 0.0;
//     new_pose[2] = 1.0;
//     new_pose[3] = 0.0;
//     new_pose[4] = 0.0;
//     new_pose[5] = 0.0;
//     new_pose[6] = 0.0;
//     new_pose[7] = 0.0;
//     new_pose[8] = 0.0;
//   } else {
//     new_pose[0] = poses_.front()[0];
//     new_pose[1] = poses_.front()[1];
//     new_pose[2] = poses_.front()[2];
//     new_pose[3] = poses_.front()[3];
//     new_pose[4] = poses_.front()[4];
//     new_pose[5] = poses_.front()[5];
//     new_pose[6] = poses_.front()[6];
//     new_pose[7] = poses_.front()[7];
//     new_pose[8] = poses_.front()[8];
//   }
//   poses_.insert(poses_.begin(),new_pose);
//   return;
// }

bool PoseGraph::GetTransform(geometry_msgs::TransformStamped& msg) {
  if (!valid_) return false;
  // Set the output
  msg = pose_;
  // Change the time stamp
  msg.header.stamp == light_data_.back().header.stamp;
  return true;
}

bool PoseGraph::Solve() {
  // Test if we have enough data
  if (light_data_.size() < window_) return true;
  // Lighthouse and pose
  DataType prev_type;
  double prev_time = 0;
  size_t counter = 0;
  // std::string next_lighthouse;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Iterators
  auto li_it = light_data_.begin();
  auto imu_it = imu_data_.begin();
  bool lastposewasimu = false;

  // Preliminary light data - first HORIZONTAL / second VERTICAL
  std::map<std::string, std::pair<hive::ViveLight*,hive::ViveLight*>> pre_data;

  std::vector<double*> new_poses;
  new_poses.push_back(new double[9]);
  lastposewasimu_ = true;
  size_t pose_index = 0;
  double bias_acc[3];
  bias_acc[0] = tracker_.acc_bias.x;
  bias_acc[1] = tracker_.acc_bias.y;
  bias_acc[2] = tracker_.acc_bias.z;
  double bias_ang[3];
  bias_ang[0] = tracker_.gyr_bias.x;
  bias_ang[1] = tracker_.gyr_bias.y;
  bias_ang[2] = tracker_.gyr_bias.z;

  while (li_it != light_data_.end()) {

    // Convert lighthouse to geometry_msgs
    geometry_msgs::Transform lhTF;
    lhTF.translation.x = environment_.lighthouses[li_it->lighthouse].translation.x;
    lhTF.translation.y = environment_.lighthouses[li_it->lighthouse].translation.y;
    lhTF.translation.z = environment_.lighthouses[li_it->lighthouse].translation.z;
    lhTF.rotation.w = environment_.lighthouses[li_it->lighthouse].rotation.w;
    lhTF.rotation.x = environment_.lighthouses[li_it->lighthouse].rotation.x;
    lhTF.rotation.y = environment_.lighthouses[li_it->lighthouse].rotation.y;
    lhTF.rotation.z = environment_.lighthouses[li_it->lighthouse].rotation.z;

    // Inertial measurements
    while(imu_it != imu_data_.end() &&
      imu_it->header.stamp < li_it->header.stamp) {
      // Time difference for iteration
      double dt;
      if (prev_type == imu)
        dt = imu_it->header.stamp.toSec() - prev_time;
      else if (prev_type == light)
        dt = 2 * (imu_it->header.stamp.toSec() - prev_time);
      else
        dt = 0.0;
      // auto prev_pose = new_poses.back();
      new_poses.push_back(new double[9]);
      size_t prev = new_poses.size() - 2;
      size_t next = new_poses.size() - 1;
      // auto next_pose = new_poses.back();
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<pgo::InertialCost, 7, 9, 9, 3, 3  >
        (new pgo::InertialCost(*imu_it,
          environment_.gravity,
          dt,
          trust_));
      // std::cout << "IMU " << imu_it->header.stamp << " - " << prev << " - " << next << " : " << dt << " - " << trust_ << std::endl;
      problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.05), new_poses[prev],
        new_poses[next],
        bias_acc,
        bias_ang);
      lastposewasimu_ = true;
      prev_type = imu;
      prev_time = imu_it->header.stamp.toSec();
      imu_it++;
    }
    // Cost related to light measurements
    // if (!lastposewasimu_) {
    //   new_poses.push_back(new double[9]);
    //   lastposewasimu_ = false;
    // }
    if (li_it->axis == HORIZONTAL) {
      ceres::DynamicAutoDiffCostFunction<pgo::ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<pgo::ViveHorizontalCost, 4>
        (new pgo::ViveHorizontalCost(*li_it,
          lhTF,
          tracker_,
          lighthouses_[li_it->lighthouse].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(9);
      hcost->SetNumResiduals(li_it->samples.size());
      size_t light_pointer = new_poses.size() - 1;
      // std::cout << "Light " << li_it->header.stamp << " - " << light_pointer << std::endl;
      problem.AddResidualBlock(hcost, new ceres::CauchyLoss(0.05), new_poses[light_pointer]);
      // Delta time
      prev_type = light;
      prev_time = li_it->header.stamp.toSec();
    } else if (li_it->axis == VERTICAL) {
      ceres::DynamicAutoDiffCostFunction<pgo::ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<pgo::ViveVerticalCost, 4>
        (new pgo::ViveVerticalCost(*li_it,
          lhTF,
          tracker_,
          lighthouses_[li_it->lighthouse].vertical_motor,
          correction_));
      vcost->AddParameterBlock(9);
      vcost->SetNumResiduals(li_it->samples.size());
      size_t light_pointer = new_poses.size() - 1;
      // std::cout << "Light " << li_it->header.stamp << " - " << light_pointer << std::endl;
      problem.AddResidualBlock(vcost, new ceres::CauchyLoss(0.05), new_poses[light_pointer]);
      // Delta time
      prev_type = light;
      prev_time = li_it->header.stamp.toSec();
    }


    // Preliminary solver data
    if (li_it->axis == HORIZONTAL)
      pre_data[li_it->lighthouse].first = &(*li_it);
    else if (li_it->axis == VERTICAL)
      pre_data[li_it->lighthouse].second = &(*li_it);
    // Solve preliminary data
    if (!valid_ && pre_data[li_it->lighthouse].first != NULL
      && pre_data[li_it->lighthouse].second != NULL) {
      double pre_pose[9];
      pre_pose[0] = 0.0;
      pre_pose[1] = 0.0;
      pre_pose[2] = 1.0;
      pre_pose[3] = 0.0;
      pre_pose[4] = 0.0;
      pre_pose[5] = 0.0;
      pre_pose[6] = 0.0;
      pre_pose[7] = 0.0;
      pre_pose[8] = 0.0;

      ceres::Problem pre_problem;
      ceres::Solver::Options pre_options;
      ceres::Solver::Summary pre_summary;
      // Horizontal data
      ceres::DynamicAutoDiffCostFunction<pgo::ViveHorizontalCost, 4> * hcost =
        new ceres::DynamicAutoDiffCostFunction<pgo::ViveHorizontalCost, 4>
        (new pgo::ViveHorizontalCost(*pre_data[li_it->lighthouse].first,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].horizontal_motor,
          correction_));
      hcost->AddParameterBlock(9);
      hcost->SetNumResiduals(pre_data[li_it->lighthouse].first->samples.size());
      pre_problem.AddResidualBlock(hcost, NULL, pre_pose);
      // Vertical data
      ceres::DynamicAutoDiffCostFunction<pgo::ViveVerticalCost, 4> * vcost =
        new ceres::DynamicAutoDiffCostFunction<pgo::ViveVerticalCost, 4>
        (new pgo::ViveVerticalCost(*pre_data[li_it->lighthouse].second,
          lhTF,
          // tracker_.imu_transform,
          tracker_,
          lighthouses_[li_it->lighthouse].vertical_motor,
          correction_));
      vcost->AddParameterBlock(9);
      vcost->SetNumResiduals(pre_data[li_it->lighthouse].second->samples.size());
      pre_problem.AddResidualBlock(vcost, NULL, pre_pose);
      pre_options.minimizer_progress_to_stdout = false;
      pre_options.max_num_iterations = 1000;
      pre_options.max_solver_time_in_seconds = 1.0;
      // std::cout << "PreSolve " << light_data_.size() << " "
      //   << pre_data[li_it->lighthouse].first->samples.size() << " "
      //   << pre_data[li_it->lighthouse].second->samples.size() << std::endl;
      ceres::Solve(pre_options, &pre_problem, &pre_summary);
      // Copy paste

      // counter = 0;
      // std::cout << "PRE_POSE" << std::endl;
      // std::cout << counter << " "
      //   << pre_summary.final_cost <<  " - "
      //   << pre_pose[0] << ", "
      //   << pre_pose[1] << ", "
      //   << pre_pose[2] << ", "
      //   << pre_pose[3] << ", "
      //   << pre_pose[4] << ", "
      //   << pre_pose[5] << ", "
      //   << pre_pose[6] << ", "
      //   << pre_pose[7] << ", "
      //   << pre_pose[8] << std::endl;
      //   counter++;

      // Check it is a good pose
      if (pre_summary.final_cost < 1e-5 * 
        (pre_data[li_it->lighthouse].second->samples.size() +
        pre_data[li_it->lighthouse].first->samples.size())) {
        // Fill poses
        while (pose_index < new_poses.size()) {
          for (size_t i = 0; i < 9; i++)
            new_poses[pose_index][i] = pre_pose[i];
          pose_index++;
        }
        // pose_index = new_poses.size()-1;
      }
    }
    li_it++;
  }

  // If we want to force the first pose to be close
  // to its previous estimate
  if (valid_) {
    for (size_t pose_idx = 0; pose_idx < poses_.size(); pose_idx++) {
      for(size_t i = 0; i < 9; i++) {
        new_poses[pose_idx][i] = poses_[pose_idx][i];
      }
    }
    for (size_t pose_idx = poses_.size(); pose_idx < new_poses.size(); pose_idx++) {
      for(size_t i = 0; i < 9; i++) {
        new_poses[pose_idx][i] = poses_.back()[i];
      }
    }
    if (first_factor_ != 0) {
      ceres::CostFunction * cost =
        new ceres::AutoDiffCostFunction<pgo::ClosenessCost, 7, 9, 9>
        (new pgo::ClosenessCost(first_factor_, 1.0));
      problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.05), poses_.front(), new_poses.front());
      problem.SetParameterBlockConstant(poses_.front());
    }
  } else if (pose_index != 0) {
      while (pose_index < new_poses.size()) {
        for (size_t i = 0; i < 9; i++)
          new_poses[pose_index][i] = new_poses.front()[i];
        pose_index++;
      }
  } else {
    return false;
  }


  // counter = 0;
  // std::cout << "PRE" << std::endl;
  // for (auto pose : new_poses) {
  //   std::cout << counter << " "
  //     << summary.final_cost <<  " - "
  //     << pose[0] << ", "
  //     << pose[1] << ", "
  //     << pose[2] << ", "
  //     << pose[3] << ", "
  //     << pose[4] << ", "
  //     << pose[5] << ", "
  //     << pose[6] << ", "
  //     << pose[7] << ", "
  //     << pose[8] << std::endl;
  //     counter++;
  // }

  options.minimizer_progress_to_stdout = false;
  options.minimizer_type = ceres::LINE_SEARCH;
  options.line_search_direction_type = ceres::LBFGS;
  if (valid_) {
    options.max_num_iterations = 500;
    options.max_solver_time_in_seconds = 5.0;
  } else {
    options.max_num_iterations = 2000;
    options.max_solver_time_in_seconds = 20.0;
  }
  // std::cout << "Solve " << new_poses.size() << " " << light_data_.size() << std::endl;
  problem.SetParameterBlockConstant(bias_acc);
  problem.SetParameterBlockConstant(bias_ang);
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;


  // counter = 0;
  // std::cout << "POS" << std::endl;
  // for (auto pose : new_poses) {
  //   std::cout << counter << " "
  //     << summary.final_cost <<  " - "
  //     << pose[0] << ", "
  //     << pose[1] << ", "
  //     << pose[2] << ", "
  //     << pose[3] << ", "
  //     << pose[4] << ", "
  //     << pose[5] << ", "
  //     << pose[6] << ", "
  //     << pose[7] << ", "
  //     << pose[8] << std::endl;
  //     counter++;
  // }


  last_cost_ = summary.final_cost;

  // Save pose -- light frame in the vive frame
  pose_.header.stamp = light_data_.back().header.stamp;
  pose_.header.frame_id = "vive";
  pose_.child_frame_id = tracker_.serial;
  // The computed pose
  Eigen::Vector3d vPi(new_poses.back()[0],
    new_poses.back()[1],
    new_poses.back()[2]);
  Eigen::Vector3d vAi(new_poses.back()[6],
    new_poses.back()[7],
    new_poses.back()[8]);
  Eigen::AngleAxisd vAAi(vAi.norm(), vAi.normalized());
  Eigen::Matrix3d vRi = vAAi.toRotationMatrix();
  // Imu to light
  Eigen::Vector3d tPi(tracker_.imu_transform.translation.x,
    tracker_.imu_transform.translation.y,
    tracker_.imu_transform.translation.z);
  Eigen::Quaterniond tQi(tracker_.imu_transform.rotation.w,
    tracker_.imu_transform.rotation.x,
    tracker_.imu_transform.rotation.y,
    tracker_.imu_transform.rotation.z);
  Eigen::Matrix3d tRi = tQi.toRotationMatrix();

  // Convert frames
  Eigen::Vector3d vPt = vRi * ( - tRi.transpose() * tPi ) + vPi;
  Eigen::Matrix3d vRt = vRi * tRi.transpose();
  Eigen::Quaterniond vQt(vRt);
  // Save to ROS msg
  pose_.transform.translation.x = vPt(0);
  pose_.transform.translation.y = vPt(1);
  pose_.transform.translation.z = vPt(2);
  pose_.transform.rotation.w = vQt.w();
  pose_.transform.rotation.x = vQt.x();
  pose_.transform.rotation.y = vQt.y();
  pose_.transform.rotation.z = vQt.z();

  if (!Valid()) return false;

  poses_.clear();
  for (size_t i = 0; i < new_poses.size(); i++)
    poses_.push_back(new_poses[i]);

  return true;
}

void PoseGraph::PrintState() {
  // if (!valid_) {
  //   std::cout << "NOT VALID" << std::endl;
  //   return;
  // }
  std::cout << last_cost_ <<  " - "
    << poses_.back()[0] << ", "
    << poses_.back()[1] << ", "
    << poses_.back()[2] << ", "
    << poses_.back()[6] << ", "
    << poses_.back()[7] << ", "
    << poses_.back()[8] << std::endl;
  return;
}


// int main(int argc, char ** argv) {
//   // ros intializations
//   std::map<std::string, PoseGraph> smap;
//   Calibration cal;
//   rosbag::View view;
//   rosbag::Bag rbag;

//   if (argc < 2) {
//     std::cout << "Usage: ... hive_offset name_of_read_bag.bag "
//       << std::endl;
//     return -1;
//   }

//   // Calibration
//   if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
//     ROS_FATAL("Can't find calibration file.");
//     return -1;
//   } else {
//     ROS_INFO("Read calibration file.");
//   }

//   rbag.open(argv[1], rosbag::bagmode::Read);
//   // Lighthouses
//   rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
//   for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
//     const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
//       bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
//     cal.SetLighthouses(*vl);
//   }
//   ROS_INFO("Lighthouses' setup complete.");

//   // Trackers
//   rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
//   for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
//     const hive::ViveCalibrationTrackerArray::ConstPtr vt =
//       bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
//     cal.SetTrackers(*vt);
//     for (auto tr : vt->trackers) {
//       smap[tr.serial] = PoseGraph(cal.environment,
//         cal.trackers[tr.serial],
//         cal.lighthouses,
//         4, TRUST, true, true);
//     }
//   }
//   ROS_INFO("Trackers' setup complete.");

//   size_t counter = 0;
//   // Light data
//   std::vector<std::string> run_topics; 
//   run_topics.push_back("/loc/vive/light");
//   run_topics.push_back("/loc/vive/imu/");
//   run_topics.push_back("/loc/vive/imu");
//   rosbag::View view_li(rbag, rosbag::TopicQuery(run_topics));
//   for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
//     const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
//     if (vl != NULL) {
//       // std::cout << "LIGHT" << std::endl;
//       smap[vl->header.frame_id].ProcessLight(vl);
//       smap[vl->header.frame_id].PrintState();
//       counter++;
//     }
//     const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
//     if (vi != NULL) {
//       // std::cout << "IMU" << std::endl;
//       smap[vi->header.frame_id].ProcessImu(vi);
//       counter++;
//     }
//     // if (counter >= 200) break;
//   }
//   ROS_INFO("Data processment complete.");

//   return 0;
// }