#ifndef HIVE_VIVE_SOLVER_H_
#define HIVE_VIVE_SOLVER_H_

#include <hive/vive.h>

class Solver {
public:
  // Solver() {}
  // Solver(Environment environment,
  //   Tracker tracker);
  // ~Solver() {};
  virtual void ProcessLight(const hive::ViveLight::ConstPtr & msg) = 0;
  virtual bool GetTransform(geometry_msgs::TransformStamped & msg) = 0;
};

#endif // HIVE_VIVE_SOLVER_H