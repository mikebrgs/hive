#include <hive/vive_refine.h>

#define ROTATION_COST_FACTOR 1.0

typedef geometry_msgs::TransformStamped TF;
typedef std::vector<TF> TFs;
typedef std::map<std::string, std::pair<hive::ViveLight*,
  hive::ViveLight*>> LightMap;

namespace refine {
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
    // std::cout << "vPi"
      // << vPi(0) << ", "
      // << vPi(1) << ", "
      // << vPi(2) << std::endl;
    // std::cout << "vRi"
      // << vRi(0,0) << ", "
      // << vRi(0,1) << ", "
      // << vRi(0,2) << std::endl;

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
    // std::cout << "vPl"
      // << vPl(0) << ", "
      // << vPl(1) << ", "
      // << vPl(2) << std::endl;
    // std::cout << "vRl"
      // << vRl(0,0) << ", "
      // << vRl(0,1) << ", "
      // << vRl(0,2) << std::endl;

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
    // std::cout << "vPi"
      // << vPi(0) << ", "
      // << vPi(1) << ", "
      // << vPi(2) << std::endl;
    // std::cout << "vRi"
      // << vRi(0,0) << ", "
      // << vRi(0,1) << ", "
      // << vRi(0,2) << std::endl;

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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

  // Inertial cost function
  class InertialCost {
  public:
    InertialCost(sensor_msgs::Imu imu,
      geometry_msgs::Vector3 gravity,
      geometry_msgs::Vector3 acc_bias,
      geometry_msgs::Vector3 gyr_bias,
      double time_step,
      double trust_weight,
      bool verbose = false);
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
    geometry_msgs::Vector3 gravity_, gyr_bias_, acc_bias_;
    // Time step and weight
    double time_step_, trust_weight_;
    // Other
    bool verbose_;
  };

  InertialCost::InertialCost(sensor_msgs::Imu imu,
    geometry_msgs::Vector3 gravity,
    geometry_msgs::Vector3 acc_bias,
    geometry_msgs::Vector3 gyr_bias,
    double time_step,
    double trust_weight,
    bool verbose) {
    imu_ = imu;
    gravity_ = gravity;
    time_step_ = time_step;
    trust_weight_ = trust_weight;
    gyr_bias_ = gyr_bias;
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
    T * residual) const {

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
    // Angle axis eigen structure
    Eigen::AngleAxis<T> prev_vAAi(prev_vAi.norm(),
      prev_vAi.normalized());
    // Eigen Quaternion
    Eigen::Quaternion<T> prev_vQi(prev_vAAi);
    // Rotation matrix
    Eigen::Matrix<T,3,3> prev_vRi;// = prev_vQi.toRotationMatrix();
    ceres::AngleAxisToRotationMatrix(&prev_vTi[6], prev_vRi.data());

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
    // Angle axis eigen structure
    Eigen::AngleAxis<T> next_vAAi(next_vAi.norm(),
      next_vAi.normalized());
    // Eigen Quaternion
    Eigen::Quaternion<T> next_vQi(next_vAAi);
    // Rotation matrix
    Eigen::Matrix<T,3,3> next_vRi;// = next_vQi.toRotationMatrix();
    ceres::AngleAxisToRotationMatrix(&next_vTi[6], next_vRi.data());

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
    Eigen::Matrix<T,3,1> iBa;
    iBa << T(acc_bias_.x),
      T(acc_bias_.y),
      T(acc_bias_.z);
    Eigen::Matrix<T,3,1> iBw;
    iBw << T(gyr_bias_.x),
      T(gyr_bias_.y),
      T(gyr_bias_.z);

    // Gravity
    Eigen::Matrix<T,3,1> vG;
    vG << T(gravity_.x),
      T(gravity_.y),
      T(gravity_.z);

    // Inertial predictions
    // Position prediction
    // T dt = T(0.0);
    T dt = T(time_step_);
    Eigen::Matrix<T,3,1> est_vPi;
    est_vPi = prev_vPi + T(dt) * prev_vVi + T(0.5 * dt * dt) * (vG - prev_vRi * (iA-iBa));
    // Velocity prediction
    Eigen::Matrix<T,3,1> est_vVi;
    est_vVi = prev_vVi + T(dt) * (vG - (prev_vRi * (iA - iBa)));

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
    text_vQi = trev_vQi + T(dt) * T(0.5) * Omega * (iW - iBw);
    // text_vQi.normalize(); // maybe remove
    // Fix small errors
    Eigen::Quaternion<T> est_vQi(text_vQi(0),
      text_vQi(1),
      text_vQi(2),
      text_vQi(3));
    Eigen::Matrix<T,3,3> est_vRi = est_vQi.toRotationMatrix();

    // Estimated from inertia vs next
    Eigen::Matrix<T,3,1> dP;
    Eigen::Matrix<T,3,1> dV;
    Eigen::Matrix<T,3,3> dR;
    dP = next_vPi - est_vPi;
    dV = next_vVi - est_vVi;
    dR = next_vRi.transpose() * est_vRi;

    // Position cost
    residual[0] = T(trust_weight_) * dP(0);
    residual[1] = T(trust_weight_) * dP(1);
    residual[2] = T(trust_weight_) * dP(2);
    // Velocity cost
    // residual[3] = prev_vPi(0) - next_vPi(0);
    // residual[4] = prev_vPi(1) - next_vPi(1);
    // residual[5] = prev_vPi(2) - next_vPi(2);
    residual[3] = T(trust_weight_) * dV(0);
    residual[4] = T(trust_weight_) * dV(1);
    residual[5] = T(trust_weight_) * dV(2);
    // Orientation cost
    T aa[3];
    ceres::RotationMatrixToAngleAxis(dR.data(), aa);
    // std::cout << aa[0] << ", " << aa[1] << ", " << aa[2] << std::endl;
    residual[6] = T(trust_weight_) *
      sqrt(1e-3 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); // WATCH OUT FOR THIS
      // Watch out for bias
    return true;
  }


  // Light cost - Cost using the poses from the imu frame to the vive frame
  class ViveCalibrationHorizontalCost
  {
  public:
    // Constructor
    ViveCalibrationHorizontalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveCalibrationHorizontalCost();
    // Ceres operator - First parameters are Vive Pose, second are lighthouse pose
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
  };

  ViveCalibrationHorizontalCost::ViveCalibrationHorizontalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    return;
  }

  ViveCalibrationHorizontalCost::~ViveCalibrationHorizontalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveCalibrationHorizontalCost::operator()(const T* const * parameters,
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
    vPl << parameters[1][0],
      parameters[1][1],
      parameters[1][2];
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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

  // Light cost - Cost using the poses from the imu frame to the vive frame
  class ViveCalibrationVerticalCost
  {
  public:
    // Constructor
    ViveCalibrationVerticalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction);
    // Destructor
    ~ViveCalibrationVerticalCost();
    // Ceres operator - First parameters are Vive Pose, second are lighthouse pose
    template <typename T>
    bool operator()(const T* const * parameters, T* residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
  };

  ViveCalibrationVerticalCost::ViveCalibrationVerticalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    lighthouse_ = lighthouse;
    correction_ = correction;
    tracker_ = tracker;
    data_ = data;
    return;
  }

  ViveCalibrationVerticalCost::~ViveCalibrationVerticalCost() {
    // Do nothing
    return;
  }

  template <typename T>
  bool ViveCalibrationVerticalCost::operator()(const T* const * parameters,
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
    vPl << parameters[1][0],
      parameters[1][1],
      parameters[1][2];
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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


  // How close the poses should be to each other
  class SmoothingCost {
  public:
    // Constructor to pass data
    // A good smoothing factor is 0.1, but it may be changed
    explicit SmoothingCost(double smoothing,
      double rotation_factor);

    // Function for ceres solver with parameters (different frames)
    template <typename T> bool operator()(const T* const prev_vTt,
      const T* const next_vTt,
      T * residual) const;

    template <typename T> bool operator()(const T* const prev_lTt,
      const T* const prev_vTl,
      const T* const next_lTt,
      const T* const next_vTl,
      T * residual) const;

    // Function for ceres solver with parameters (same frame)
    template <typename T> bool operator()(const T* const prev_lTt,
      const T* const next_lTt,
      const T* const vTl,
      T * residual) const;

  private:
    double smoothing_;
    double rotation_factor_;
  };

  // Constructor to pass data
  SmoothingCost::SmoothingCost(double smoothing,
    double rotation_factor) {
    smoothing_ = smoothing;
    rotation_factor_ = rotation_factor;
  }

  // Function for ceres solver with parameters
  template <typename T> bool SmoothingCost::operator()(const T* const prev_vTt,
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
    Eigen::Matrix<T, 3, 3> prev_vRt;
    ceres::AngleAxisToRotationMatrix(&prev_vTt[3], prev_vRt.data());
    Eigen::Matrix<T, 3, 3> next_vRt;
    ceres::AngleAxisToRotationMatrix(&next_vTt[3], next_vRt.data());

    // // Translation cost with smoothing
    residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
    residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
    residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
    // // Rotation cost with smoothing
    T aa[3];
    Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
    ceres::RotationMatrixToAngleAxis(R.data(), aa);
    residual[3] = T(rotation_factor_) * T(smoothing_)*
      sqrt(1e-3 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
    return true;
  }

  // Function for ceres solver with parameters
  template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
    const T* const prev_vTl,
    const T* const next_lTt,
    const T* const next_vTl,
    T * residual) const {
    // cost function here

    // Frame conversion
    Eigen::Matrix<T, 3, 1> prev_lPt;
    prev_lPt << prev_lTt[0],
      prev_lTt[1],
      prev_lTt[2];
    Eigen::Matrix<T, 3, 1> next_lPt;
    next_lPt << next_lTt[0],
      next_lTt[1],
      next_lTt[2];
    Eigen::Matrix<T, 3, 3> prev_lRt;
    ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
    Eigen::Matrix<T, 3, 3> next_lRt;
    ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

    Eigen::Matrix<T, 3, 1> prev_vPl;
    prev_vPl << prev_vTl[0],
      prev_vTl[1],
      prev_vTl[2];
    Eigen::Matrix<T, 3, 1> next_vPl;
    next_vPl << next_vTl[0],
      next_vTl[1],
      next_vTl[2];
    Eigen::Matrix<T, 3, 3> prev_vRl;
    ceres::AngleAxisToRotationMatrix(&prev_vTl[3], prev_vRl.data());
    Eigen::Matrix<T, 3, 3> next_vRl;
    ceres::AngleAxisToRotationMatrix(&next_vTl[3], next_vRl.data());

    Eigen::Matrix<T, 3, 1> prev_vPt = prev_vRl * prev_lPt + prev_vPl;
    Eigen::Matrix<T, 3, 3> prev_vRt = prev_vRl * prev_lRt;

    Eigen::Matrix<T, 3, 1> next_vPt = next_vRl * next_lPt + next_vPl;
    Eigen::Matrix<T, 3, 3> next_vRt = next_vRl * next_lRt;

    // // Translation cost with smoothing
    residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
    residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
    residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
    // // Rotation cost with smoothing
    T aa[3];
    Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
    ceres::RotationMatrixToAngleAxis(R.data(), aa);
    residual[3] = T(rotation_factor_) * T(smoothing_)*
      sqrt(1e-3 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
    return true;
  }

  template <typename T> bool SmoothingCost::operator()(const T* const prev_lTt,
    const T* const next_lTt,
    const T* const vTl,
    T * residual) const {
    // Frame conversion
    Eigen::Matrix<T, 3, 1> prev_lPt;
    prev_lPt << prev_lTt[0],
      prev_lTt[1],
      prev_lTt[2];
    Eigen::Matrix<T, 3, 1> next_lPt;
    next_lPt << next_lTt[0],
      next_lTt[1],
      next_lTt[2];
    Eigen::Matrix<T, 3, 3> prev_lRt;
    ceres::AngleAxisToRotationMatrix(&prev_lTt[3], prev_lRt.data());
    Eigen::Matrix<T, 3, 3> next_lRt;
    ceres::AngleAxisToRotationMatrix(&next_lTt[3], next_lRt.data());

    Eigen::Matrix<T, 3, 1> vPl;
    vPl << vTl[0],
      vTl[1],
      vTl[2];
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&vTl[3], vRl.data());

    Eigen::Matrix<T, 3, 1> prev_vPt = vRl * prev_lPt + vPl;
    Eigen::Matrix<T, 3, 3> prev_vRt = vRl * prev_lRt;

    Eigen::Matrix<T, 3, 1> next_vPt = vRl * next_lPt + vPl;
    Eigen::Matrix<T, 3, 3> next_vRt = vRl * next_lRt;

    // // Translation cost with smoothing
    residual[0] = T(smoothing_) * (prev_vPt(0) - next_vPt(0));
    residual[1] = T(smoothing_) * (prev_vPt(1) - next_vPt(1));
    residual[2] = T(smoothing_) * (prev_vPt(2) - next_vPt(2));
    // // Rotation cost with smoothing
    T aa[3];
    Eigen::Matrix<T, 3, 3> R = next_vRt.transpose() * prev_vRt;
    ceres::RotationMatrixToAngleAxis(R.data(), aa);
    residual[3] = T(rotation_factor_) * T(smoothing_) *
      sqrt(1e-3 + aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
    return true;
  }


  class PoseHorizontalCost {
  public:
    explicit PoseHorizontalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction);

    template <typename T> bool operator()(const T* const * parameters,
      T * residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
  };

  class PoseVerticalCost {
  public:
    explicit PoseVerticalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction);

    template <typename T> bool operator()(const T* const * parameters,
      T * residual) const;
  private:
    bool correction_;
    Tracker tracker_;
    Motor lighthouse_;
    hive::ViveLight data_;
  };

  PoseHorizontalCost::PoseHorizontalCost(hive::ViveLight data,
      Tracker tracker,
      Motor lighthouse,
      bool correction) {
      data_ = data;
      correction_ = correction;
      tracker_ = tracker;
      lighthouse_ = lighthouse;
    }

  template <typename T> bool PoseHorizontalCost::operator()(const T* const * parameters,
    T * residual) const {
    // Pose of the tracker in the vive frame
    Eigen::Matrix<T, 3, 1> vPt;
    vPt << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRt;
    ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());
    // Pose of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << parameters[1][0],
      parameters[1][1],
      parameters[1][2];
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

    Eigen::Matrix<T, 3, 1> lPt = vRl.transpose() * vPt - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRt = vRl.transpose() * vRt;

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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

  // template <typename T> bool PoseHorizontalCost::operator()(const T* const * parameters,
  //   T * residual) const {
  //   Eigen::Matrix<T, 3, 1> lPt;
  //   lPt << parameters[0][0],
  //     parameters[0][1],
  //     parameters[0][2];
  //   Eigen::Matrix<T, 3, 3> lRt;
  //   ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

  //   size_t counter = 0;
  //   for (auto li_it = data_.samples.begin();
  //     li_it != data_.samples.end(); li_it++) {
  //     auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
  //     if (sensor_it == tracker_.sensors.end()) return false;
  //     Eigen::Matrix<T, 3, 1> tPs;
  //     tPs << T(sensor_it->second.position.x),
  //       T(sensor_it->second.position.y),
  //       T(sensor_it->second.position.z);

  //     Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

  //     T ang; // The final angle
  //     T x = (lPs(0)/lPs(2)); // Horizontal angle
  //     T y = (lPs(1)/lPs(2)); // Vertical angle
  //     T phase = T(lighthouse_.phase);
  //     T tilt = T(lighthouse_.tilt);
  //     T gib_phase = T(lighthouse_.gib_phase);
  //     T gib_mag = T(lighthouse_.gib_magnitude);
  //     T curve = T(lighthouse_.curve);

  //     if (correction_) {
  //       ang = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
  //     } else {
  //       ang = atan(x);
  //     }


  //     residual[counter] = T(li_it->angle) - ang;
  //     counter++;
  //   }
  //   return true;
  // }

  PoseVerticalCost::PoseVerticalCost(hive::ViveLight data,
    Tracker tracker,
    Motor lighthouse,
    bool correction) {
    data_ = data;
    correction_ = correction;
    tracker_ = tracker;
    lighthouse_ = lighthouse;
  }

  template <typename T> bool PoseVerticalCost::operator()(const T* const * parameters,
    T * residual) const {
    // Pose of the tracker in the vive frame
    Eigen::Matrix<T, 3, 1> vPt;
    vPt << parameters[0][0],
      parameters[0][1],
      parameters[0][2];
    Eigen::Matrix<T, 3, 3> vRt;
    ceres::AngleAxisToRotationMatrix(&parameters[0][3], vRt.data());
    // Pose of the lighthouse in the vive frame
    Eigen::Matrix<T, 3, 1> vPl;
    vPl << parameters[1][0],
      parameters[1][1],
      parameters[1][2];
    Eigen::Matrix<T, 3, 3> vRl;
    ceres::AngleAxisToRotationMatrix(&parameters[1][3], vRl.data());

    Eigen::Matrix<T, 3, 1> lPt = vRl.transpose() * vPt - vRl.transpose() * vPl;
    Eigen::Matrix<T, 3, 3> lRt = vRl.transpose() * vRt;

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
      T phase = T(SCALE_PHASE * lighthouse_.phase);
      T tilt = T(SCALE_TILT * lighthouse_.tilt);
      T curve = T(SCALE_CURVE * lighthouse_.curve);
      T gib_mag = T(SCALE_GIB * lighthouse_.gib_magnitude);
      T gib_phase = T(lighthouse_.gib_phase);

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

  // template <typename T> bool PoseVerticalCost::operator()(const T* const * parameters,
  //   T * residual) const {
  //   Eigen::Matrix<T, 3, 1> lPt;
  //   lPt << parameters[0][0],
  //     parameters[0][1],
  //     parameters[0][2];
  //   Eigen::Matrix<T, 3, 3> lRt;
  //   ceres::AngleAxisToRotationMatrix(&parameters[0][3], lRt.data());

  //   size_t counter = 0;
  //   for (auto li_it = data_.samples.begin();
  //     li_it != data_.samples.end(); li_it++) {
  //     auto sensor_it = tracker_.sensors.find((uint8_t)li_it->sensor);
  //     if (sensor_it == tracker_.sensors.end()) return false;
  //     Eigen::Matrix<T, 3, 1> tPs;
  //     tPs << T(sensor_it->second.position.x),
  //       T(sensor_it->second.position.y),
  //       T(sensor_it->second.position.z);

  //     Eigen::Matrix<T, 3, 1> lPs = lRt * tPs + lPt;

  //     T ang; // The final angle
  //     T x = (lPs(0)/lPs(2)); // Horizontal angle
  //     T y = (lPs(1)/lPs(2)); // Vertical angle
  //     T phase = T(lighthouse_.phase);
  //     T tilt = T(lighthouse_.tilt);
  //     T gib_phase = T(lighthouse_.gib_phase);
  //     T gib_mag = T(lighthouse_.gib_magnitude);
  //     T curve = T(lighthouse_.curve);

  //     if (correction_) {
  //       ang = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
  //     } else {
  //       ang = atan(y);
  //     }

  //     residual[counter] = T(li_it->angle) - ang;
  //     counter++;
  //   }
  //   return true;
  // }

}

Refinery::Refinery(Calibration & calibration) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = true;
  inertial_ = false;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  inertial_ = false;
  smoothing_ = 0.0;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction,
  double smoothing) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = smoothing;
  inertial_ = false;
  return;
}

Refinery::Refinery(Calibration & calibration,
  bool correction,
  double smoothing,
  bool inertial) {
  // Initialize calibration (reference)
  calibration_ = calibration;
  correction_ = correction;
  smoothing_ = smoothing;
  inertial_ = inertial;
  return;
}

Refinery::~Refinery() {
  // pass
}

bool Refinery::AddImu(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg == NULL) return false;
  // New non const structure
  sensor_msgs::Imu * clone_msg = new sensor_msgs::Imu(*msg);
  // Save
  data_[msg->header.frame_id].second.push_back(*clone_msg);

  return true;
}

bool Refinery::AddLight(const hive::ViveLight::ConstPtr& msg) {
  if (msg == NULL) return false;
  // New non const structure
  hive::ViveLight * clone_msg = new hive::ViveLight(*msg);
  // Removing outliers
  auto sample_it = clone_msg->samples.begin();
  while (sample_it != clone_msg->samples.end()) {
    if (sample_it->angle > -M_PI/3.0 && sample_it->angle < M_PI / 3.0) {
      sample_it++;
    } else {
      clone_msg->samples.erase(sample_it);
    }
  }
  // If empty do not use it
  if (clone_msg->samples.size() == 0) {
    clone_msg++;
    return false;
  }
  // Save
  data_[msg->header.frame_id].first.push_back(*clone_msg);

  return true;
}

bool Refinery::Solve() {
  if (inertial_)
    return SolveInertial();
  return SolveStatic();
}

typedef std::map<std::string, std::pair<hive::ViveLight*,hive::ViveLight*>> LightPointerMapPair;
typedef std::map<std::string, std::vector<double*>> PoseVectorMap;
typedef std::map<std::string, double*> PoseMap;

bool Refinery::SolveInertial() {
  std::cout << "Solving Intertial" << std::endl;
  // All the poses
  PoseVectorMap poses;
  PoseMap lighthouses;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Initialize lighthouses
  for (auto lighthouse : calibration_.environment.lighthouses) {
    lighthouses[lighthouse.first] = new double[6];
    // Position
    lighthouses[lighthouse.first][0] =
      calibration_.environment.lighthouses[lighthouse.first].translation.x;
    lighthouses[lighthouse.first][1] =
      calibration_.environment.lighthouses[lighthouse.first].translation.y;
    lighthouses[lighthouse.first][2] =
      calibration_.environment.lighthouses[lighthouse.first].translation.z;
    // Orientation
    Eigen::Quaterniond vQl(
      calibration_.environment.lighthouses[lighthouse.first].rotation.w,
      calibration_.environment.lighthouses[lighthouse.first].rotation.x,
      calibration_.environment.lighthouses[lighthouse.first].rotation.y,
      calibration_.environment.lighthouses[lighthouse.first].rotation.z);
    Eigen::AngleAxisd vAAl(vQl);
    lighthouses[lighthouse.first][3] = vAAl.angle() * vAAl.axis()(0);
    lighthouses[lighthouse.first][4] = vAAl.angle() * vAAl.axis()(1);
    lighthouses[lighthouse.first][5] = vAAl.angle() * vAAl.axis()(2);
  }

  std::cout << "Reading...\n" << std::flush;
  for (auto tracker_data : data_) {
    // std::cout << "Tracker " << tracker_data.first << std::endl;
    // More readable structures
    SweepVec light_data = tracker_data.second.first;
    ImuVec imu_data = tracker_data.second.second;
    Tracker tracker = calibration_.trackers[tracker_data.first];

    // Other initializations
    bool lastposewasimu = false;
    ros::Time prev_time;

    // Auxiliary structures
    // LightPointerMapPair pre_data;
    std::vector<hive::ViveLight> pre_data;

    // Pose pointers
    double * next_pose = nullptr;
    double * prev_pose = nullptr;

    // Initialize iterators
    auto li_it = light_data.begin();
    auto imu_it = imu_data.begin();
    // auto pose_it = poses[tracker.serial].begin();
    // TODO change pose iterator
    size_t pose_it = 0;
    // Iterate light data
    poses[tracker.serial].push_back(new double[9]);
    while (li_it != light_data.end()) {
      // std::cout << "Light " << li_it->lighthouse << std::endl;

      // Iterate imu data
      if (pre_data.size() >= 4) {
        while (imu_it != imu_data.end()
          && imu_it->header.stamp < li_it->header.stamp) {
          // std::cout << "IMU " << li_it->lighthouse << std::endl;
          // New pose for imu data
          poses[tracker.serial].push_back(new double[9]);
          prev_pose = next_pose;
          next_pose = poses[tracker.serial].back();
          lastposewasimu = true;
          // Time delta
          double dt = (imu_it->header.stamp - prev_time).toSec();
          if (!lastposewasimu)
            dt = 2*dt;
          // Cost related to inertial measurements
          // std::cout << "InertialCost " << prev_counter << " " << next_counter << std::endl;
          ceres::CostFunction * cost =
            new ceres::AutoDiffCostFunction<refine::InertialCost, 7, 9, 9>
            (new refine::InertialCost(*imu_it,
              calibration_.environment.gravity,
              tracker.acc_bias,
              tracker.gyr_bias,
              dt,
              smoothing_));
          problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
              poses[tracker.serial][poses[tracker.serial].size()-2],
              poses[tracker.serial][poses[tracker.serial].size()-1]);
          std::cout << "." << std::flush;

          prev_time = imu_it->header.stamp;

          // Next imu msg
          imu_it++;
        }
      }

      // Save data
      pre_data.push_back(*li_it);
      while (pre_data.size() > 4)
        pre_data.erase(pre_data.begin());
      // if (li_it->axis == HORIZONTAL)
      //   pre_data[li_it->lighthouse].first = &(*li_it);
      // else if (li_it->axis == VERTICAL)
      //   pre_data[li_it->lighthouse].second = &(*li_it);
      // Initialize poses for solver
      // if (pre_data[li_it->lighthouse].first != NULL
      //   && pre_data[li_it->lighthouse].second != NULL) {
      if (pre_data.size() == 4) {
        ceres::Problem pre_problem;
        ceres::Solver::Options pre_options;
        ceres::Solver::Summary pre_summary;

        if (pose_it == 0) {
          poses[tracker.serial].back()[0] = 0.0;
          poses[tracker.serial].back()[1] = 0.0;
          poses[tracker.serial].back()[2] = 1.0;
          poses[tracker.serial].back()[3] = 0.0;
          poses[tracker.serial].back()[4] = 0.0;
          poses[tracker.serial].back()[5] = 0.0;
          poses[tracker.serial].back()[6] = 0.0;
          poses[tracker.serial].back()[7] = 0.0;
          poses[tracker.serial].back()[8] = 0.0;
        } else {
          poses[tracker.serial].back()[0] = 
            poses[tracker.serial][pose_it-1][0];
          poses[tracker.serial].back()[1] = 
            poses[tracker.serial][pose_it-1][1];
          poses[tracker.serial].back()[2] = 
            poses[tracker.serial][pose_it-1][2];
          poses[tracker.serial].back()[3] = 
            poses[tracker.serial][pose_it-1][3];
          poses[tracker.serial].back()[4] = 
            poses[tracker.serial][pose_it-1][4];
          poses[tracker.serial].back()[5] = 
            poses[tracker.serial][pose_it-1][5];
          poses[tracker.serial].back()[6] = 
            poses[tracker.serial][pose_it-1][6];
          poses[tracker.serial].back()[7] = 
            poses[tracker.serial][pose_it-1][7];
          poses[tracker.serial].back()[8] = 
            poses[tracker.serial][pose_it-1][8];
        }
        // Lighthouse
        // std::cout << pre_summary.final_cost << " - "
        //   << poses[tracker.serial].back()[0] << ", "
        //   << poses[tracker.serial].back()[1] << ", "
        //   << poses[tracker.serial].back()[2] << ", "
        //   << poses[tracker.serial].back()[3] << ", "
        //   << poses[tracker.serial].back()[4] << ", "
        //   << poses[tracker.serial].back()[5] << ", "
        //   << poses[tracker.serial].back()[6] << ", "
        //   << poses[tracker.serial].back()[7] << ", "
        //   << poses[tracker.serial].back()[8] << std::endl;

        // Horizontal data
        double sample_counter = 0.0;
        for (auto light : pre_data) {
          geometry_msgs::Transform lighthouse;
          lighthouse.translation =
            calibration_.environment.lighthouses[light.lighthouse].translation;
          lighthouse.rotation =
            calibration_.environment.lighthouses[light.lighthouse].rotation;
          if (light.axis == HORIZONTAL) {
            ceres::DynamicAutoDiffCostFunction<refine::ViveHorizontalCost, 4> * hcost =
              new ceres::DynamicAutoDiffCostFunction<refine::ViveHorizontalCost, 4>
              (new refine::ViveHorizontalCost(light,
                lighthouse,
                tracker,
                calibration_.lighthouses[light.lighthouse].horizontal_motor,
                correction_));
            hcost->AddParameterBlock(9);
            hcost->SetNumResiduals(light.samples.size());
            pre_problem.AddResidualBlock(hcost, NULL, poses[tracker.serial].back());
            sample_counter += light.samples.size();
          } else if (light.axis == VERTICAL) {
            // Vertical data
            ceres::DynamicAutoDiffCostFunction<refine::ViveVerticalCost, 4> * vcost =
              new ceres::DynamicAutoDiffCostFunction<refine::ViveVerticalCost, 4>
              (new refine::ViveVerticalCost(light,
                lighthouse,
                tracker,
                calibration_.lighthouses[light.lighthouse].vertical_motor,
                correction_));
            vcost->AddParameterBlock(9);
            vcost->SetNumResiduals(light.samples.size());
            pre_problem.AddResidualBlock(vcost, NULL, poses[tracker.serial].back());
            sample_counter += light.samples.size();
          }
        }

        // Solve
        pre_options.minimizer_progress_to_stdout = false;
        pre_options.max_solver_time_in_seconds = 1.0;
        pre_options.max_num_iterations = 1000;
        ceres::Solve(pre_options, &pre_problem, &pre_summary);

        // std::cout << li_it->lighthouse << " - "
        //   << lighthouse.translation.x << ", "
        //   << lighthouse.translation.y << ", "
        //   << lighthouse.translation.z << ", "
        //   << lighthouse.rotation.w << ", "
        //   << lighthouse.rotation.x << ", "
        //   << lighthouse.rotation.y << ", "
        //   << lighthouse.rotation.z << std::endl;

        // std::cout << pre_summary.final_cost << " - "
        //   << poses[tracker.serial].back()[0] << ", "
        //   << poses[tracker.serial].back()[1] << ", "
        //   << poses[tracker.serial].back()[2] << ", "
        //   << poses[tracker.serial].back()[3] << ", "
        //   << poses[tracker.serial].back()[4] << ", "
        //   << poses[tracker.serial].back()[5] << ", "
        //   << poses[tracker.serial].back()[6] << ", "
        //   << poses[tracker.serial].back()[7] << ", "
        //   << poses[tracker.serial].back()[8] << std::endl;

        // Copy paste
        if (pre_summary.final_cost < 1e-5 * sample_counter) {
          while (pose_it < poses[tracker.serial].size()) {
            for (size_t i = 0; i < 9; i++) {
              poses[tracker.serial][pose_it][i] = poses[tracker.serial].back()[i];
            }
            pose_it++;
          }
        } else {
          li_it++;
          pre_data.pop_back();
          continue;
        }
      } else {
        li_it++;
        continue;
      }

      // Cost related to light measurements
      if (li_it->axis == HORIZONTAL) {
        // std::cout << "ViveCalibrationHorizontalCost " << next_counter << std::endl;
        ceres::DynamicAutoDiffCostFunction<refine::ViveCalibrationHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<refine::ViveCalibrationHorizontalCost, 4>
          (new refine::ViveCalibrationHorizontalCost(*li_it,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(9);
        hcost->AddParameterBlock(6);
        hcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(hcost, new ceres::CauchyLoss(0.5),
          poses[tracker.serial][poses[tracker.serial].size()-1],
          lighthouses[li_it->lighthouse]);
      } else if (li_it->axis == VERTICAL) {
        // std::cout << "ViveCalibrationVerticalCost " << next_counter << std::endl;
        ceres::DynamicAutoDiffCostFunction<refine::ViveCalibrationVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<refine::ViveCalibrationVerticalCost, 4>
          (new refine::ViveCalibrationVerticalCost(*li_it,
            tracker,
            calibration_.lighthouses[li_it->lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(9);
        vcost->AddParameterBlock(6);
        vcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(vcost, new ceres::CauchyLoss(0.5),
          poses[tracker.serial][poses[tracker.serial].size()-1],
          lighthouses[li_it->lighthouse]);
      }
      std::cout << "*" << std::flush;
      prev_time = li_it->header.stamp;
      // Next light msg
      li_it++;
    }
    // End of light_it
  }
  std::cout << std::endl;
  // End of tracker_data

  // Fix one of the lighthouses to the vive frame
  problem.SetParameterBlockConstant(lighthouses.begin()->second);

  PoseVectorMap clone_poses(poses);
  PoseMap clone_lighthouses(lighthouses);

  // Solver's options
  options.minimizer_progress_to_stdout = true;
  // options.minimizer_type = ceres::LINE_SEARCH;
  // options.line_search_direction_type = ceres::LBFGS;
  options.max_num_iterations = REFINE_ITERATIONS; // TODO change this

  // std::cout << "PREV Tr:" << std::endl;
  // for (auto tr_it = clone_poses.begin(); tr_it != clone_poses.end(); tr_it++) {
  //   for (auto po_it = tr_it->second.begin(); po_it != tr_it->second.end(); po_it++) {
  //     std::cout <<  (*po_it)[0] << ", "
  //       << (*po_it)[1] << ", "
  //       << (*po_it)[2] << ", "
  //       << (*po_it)[6] << ", "
  //       << (*po_it)[7] << ", "
  //       << (*po_it)[8] << std::endl;
  //   }
  // }

  std::cout << "PREV Lh:" << std::endl;
  for (auto lh_it = clone_lighthouses.begin(); lh_it != clone_lighthouses.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }

  ceres::Solve(options, &problem, &summary);


  // std::cout << "NEW Tr:" << std::endl;
  // for (auto tr_it = poses.begin(); tr_it != poses.end(); tr_it++) {
  //   for (auto po_it = tr_it->second.begin(); po_it != tr_it->second.end(); po_it++) {
  //     std::cout <<  (*po_it)[0] << ", "
  //       << (*po_it)[1] << ", "
  //       << (*po_it)[2] << ", "
  //       << (*po_it)[6] << ", "
  //       << (*po_it)[7] << ", "
  //       << (*po_it)[8] << std::endl;
  //   }
  // }
  std::cout << "NEW Lh:" << std::endl;
  for (auto lh_it = lighthouses.begin(); lh_it != lighthouses.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }

  std::cout << summary.BriefReport() << std::endl;

  // Save all the new lighthouse poses
  for (auto lighthouse : lighthouses) {
    calibration_.environment.lighthouses[lighthouse.first].translation.x
      = lighthouse.second[0];
    calibration_.environment.lighthouses[lighthouse.first].translation.y
      = lighthouse.second[1];
    calibration_.environment.lighthouses[lighthouse.first].translation.z
      = lighthouse.second[2];
    Eigen::Vector3d vAl(lighthouse.second[3],
      lighthouse.second[4],
      lighthouse.second[5]);
    Eigen::AngleAxisd vAAl;
    if (vAl.norm() != 0)
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl.normalized());
    else 
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl);
    Eigen::Quaterniond vQl(vAAl);
    calibration_.environment.lighthouses[lighthouse.first].rotation.w
      = vQl.w();
    calibration_.environment.lighthouses[lighthouse.first].rotation.x
      = vQl.x();
    calibration_.environment.lighthouses[lighthouse.first].rotation.y
      = vQl.y();
    calibration_.environment.lighthouses[lighthouse.first].rotation.z
      = vQl.z();
  }

  return true;
}

// Solve the problem
bool Refinery::SolveStatic() {
  // Check requirements
  if (calibration_.environment.lighthouses.size() <= 1) {
    ROS_WARN("Not enough lighthouses for Refinement.");
    return false;
  }

  // Ceres set up
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;


  // Environment transforms
  std::map<std::string, double[6]> vTl;
  for (auto lh_it = calibration_.environment.lighthouses.begin();
    lh_it != calibration_.environment.lighthouses.end(); lh_it++) {
    // std::cout << "LH: " << lh_it->first << std::endl;
    // Conversion from quaternion to angle axis
    Eigen::AngleAxisd vAl(Eigen::Quaterniond(
      lh_it->second.rotation.w,
      lh_it->second.rotation.x,
      lh_it->second.rotation.y,
      lh_it->second.rotation.z));
    // Conversion to double array
    vTl[lh_it->first][0] = lh_it->second.translation.x;
    vTl[lh_it->first][1] = lh_it->second.translation.y;
    vTl[lh_it->first][2] = lh_it->second.translation.z;
    vTl[lh_it->first][3] = vAl.axis()(0) * vAl.angle();
    vTl[lh_it->first][4] = vAl.axis()(1) * vAl.angle();
    vTl[lh_it->first][5] = vAl.axis()(2) * vAl.angle();
  }
  // first -- horizontal observations
  // second -- vertical observations
  // Vector to save the poses
  std::vector<double*> poses;
  size_t pose_pointer = 0;

  // Iterate tracker data
  for (auto tr_it = data_.begin(); tr_it != data_.end(); tr_it++) {
    LightMap observations;
    // Iterate light data
    for (auto li_it = tr_it->second.first.begin();
      li_it != tr_it->second.first.end(); li_it++) {
      // Check if lighthouse is in calibration -- if not continue
      if (vTl.find(li_it->lighthouse) == vTl.end()) {
        continue;
      }


      // Save observations
      if (li_it->axis == HORIZONTAL) {
        // Convert from iterator to pointer
        observations[li_it->lighthouse].first = &(*li_it);
      } else if (li_it->axis == VERTICAL) {
        // Convert from iterator to pointer
        observations[li_it->lighthouse].second  = &(*li_it);
      }

      TF tf;
      if (observations[li_it->lighthouse].first != NULL &&
        observations[li_it->lighthouse].second != NULL) {
        double * pose = new double[6];
        poses.push_back(pose);
        poses.back()[0] = 0;
        poses.back()[1] = 0;
        poses.back()[2] = 1;
        poses.back()[3] = 0;
        poses.back()[4] = 0;
        poses.back()[5] = 0;
        ceres::Problem pre_problem;
        // Horizontal
        ceres::DynamicAutoDiffCostFunction<refine::PoseHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<refine::PoseHorizontalCost, 4>
          (new refine::PoseHorizontalCost(*observations[li_it->lighthouse].first,
            calibration_.trackers[tr_it->first],
            calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(6);
        hcost->AddParameterBlock(6);
        hcost->SetNumResiduals(observations[li_it->lighthouse].first->samples.size());
        pre_problem.AddResidualBlock(hcost, new ceres::CauchyLoss(0.05), poses.back(),
          vTl[li_it->lighthouse]);
        // Vertical
        ceres::DynamicAutoDiffCostFunction<refine::PoseVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<refine::PoseVerticalCost, 4>
          (new refine::PoseVerticalCost(*observations[li_it->lighthouse].second,
            calibration_.trackers[tr_it->first],
            calibration_.lighthouses[li_it->lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(6);
        vcost->AddParameterBlock(6);
        vcost->SetNumResiduals(observations[li_it->lighthouse].second->samples.size());
        pre_problem.AddResidualBlock(vcost, new ceres::CauchyLoss(0.05), poses.back(),
          vTl[li_it->lighthouse]);
        // Not solving for lighthouses
        pre_problem.SetParameterBlockConstant(vTl[li_it->lighthouse]);
        // Solve
        ceres::Solve(options, &pre_problem, &summary);
        // std::cout << summary.final_cost << " - "
        //   << poses.back()[0] << ", "
        //   << poses.back()[1] << ", "
        //   << poses.back()[2] << ", "
        //   << poses.back()[3] << ", "
        //   << poses.back()[4] << ", "
        //   << poses.back()[5] << std::endl;
        // Check
        if (summary.final_cost > 1e-5 * (
          observations[li_it->lighthouse].second->samples.size() +
          observations[li_it->lighthouse].first->samples.size())) {
          poses.pop_back();
          continue;
        } else {
        }
        // Fill
        while(pose_pointer < poses.size()) {
          for (size_t i = 0; i < 6; i++) {
            poses[pose_pointer][i] = poses.back()[i];
          }
          pose_pointer++;
        }
        // pose_pointer = poses.size();
      } else {
        continue;
      }



      // Horizontal cost
      if (li_it->axis == HORIZONTAL) {
        // std::cout << "Light H " << li_it->lighthouse << std::endl;
        ceres::DynamicAutoDiffCostFunction<refine::PoseHorizontalCost, 4> * hcost =
          new ceres::DynamicAutoDiffCostFunction<refine::PoseHorizontalCost, 4>
          (new refine::PoseHorizontalCost(*li_it,
            calibration_.trackers[tr_it->first],
            calibration_.lighthouses[li_it->lighthouse].horizontal_motor,
            correction_));
        hcost->AddParameterBlock(6);
        hcost->AddParameterBlock(6);
        hcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(hcost, new ceres::CauchyLoss(0.05), poses.back(),
          vTl[li_it->lighthouse]);
      // Vertical cost
      } else if (li_it->axis == VERTICAL) {
        // std::cout << "Light V " << li_it->lighthouse << std::endl;
        ceres::DynamicAutoDiffCostFunction<refine::PoseVerticalCost, 4> * vcost =
          new ceres::DynamicAutoDiffCostFunction<refine::PoseVerticalCost, 4>
          (new refine::PoseVerticalCost(*li_it,
            calibration_.trackers[tr_it->first],
            calibration_.lighthouses[li_it->lighthouse].vertical_motor,
            correction_));
        vcost->AddParameterBlock(6);
        vcost->AddParameterBlock(6);
        vcost->SetNumResiduals(li_it->samples.size());
        problem.AddResidualBlock(vcost, new ceres::CauchyLoss(0.05), poses.back(),
          vTl[li_it->lighthouse]);
      }
      // Smoothing cost
      if (poses.size() >= 2) {
        // std::cout << "Inertial" << std::endl;
        ceres::CostFunction * cost =
          new ceres::AutoDiffCostFunction<refine::SmoothingCost, 4, 6, 6>
          (new refine::SmoothingCost(smoothing_, ROTATION_COST_FACTOR));
        problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
          poses[poses.size()-2], poses[poses.size()-1]);
      // If the poses are in different frames
      }
    }
  }


  // Fix one of the lighthouses to the vive frame
  problem.SetParameterBlockConstant(vTl.begin()->second);


  std::map<std::string, double[6]> clone_lhs(vTl);

  // Solver's options
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500; // TODO change this

  ceres::Solve(options, &problem, &summary);

  std::cout << "PREV:" << std::endl;
  for (auto lh_it = clone_lhs.begin(); lh_it != clone_lhs.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << "NEW:" << std::endl;
  for (auto lh_it = vTl.begin(); lh_it != vTl.end(); lh_it++) {
    std::cout << lh_it->first << " - "
      << lh_it->second[0] << ", "
      << lh_it->second[1] << ", "
      << lh_it->second[2] << ", "
      << lh_it->second[3] << ", "
      << lh_it->second[4] << ", "
      << lh_it->second[5] << std::endl;
  }
  std::cout << std::endl;

  std::cout << summary.BriefReport() << std::endl;

  // Save all the new lighthouse poses
  for (auto lighthouse : vTl) {
    calibration_.environment.lighthouses[lighthouse.first].translation.x
      = lighthouse.second[0];
    calibration_.environment.lighthouses[lighthouse.first].translation.y
      = lighthouse.second[1];
    calibration_.environment.lighthouses[lighthouse.first].translation.z
      = lighthouse.second[2];
    Eigen::Vector3d vAl(lighthouse.second[3],
      lighthouse.second[4],
      lighthouse.second[5]);
    Eigen::AngleAxisd vAAl;
    if (vAl.norm() != 0)
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl.normalized());
    else 
      vAAl = Eigen::AngleAxisd(vAl.norm(), vAl);
    Eigen::Quaterniond vQl(vAAl);
    calibration_.environment.lighthouses[lighthouse.first].rotation.w
      = vQl.w();
    calibration_.environment.lighthouses[lighthouse.first].rotation.x
      = vQl.x();
    calibration_.environment.lighthouses[lighthouse.first].rotation.y
      = vQl.y();
    calibration_.environment.lighthouses[lighthouse.first].rotation.z
      = vQl.z();
  }

  return true;
}

Calibration Refinery::GetCalibration() {
  return calibration_;
}