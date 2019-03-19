/**
 *
 *
 *
 *
*/

#include <hive/vive_base_solve.h>

int main(int argc, char ** argv)
{
  // Data
  Calibration cal;
  BaseMap solver;

  rosbag::Bag rbag, wbag;
  rosbag::View view;

  ros::init(argc, argv, "hive_baseline_solver");
  ros::NodeHandle nh;


  // Read bag with data
  bool write_bag;
  if (argc == 2) {
    write_bag = false;
  }
  else if (argc >= 3) {
    std::string write_bag(argv[2]);
    wbag.open(write_bag, rosbag::bagmode::Write);
    write_bag = true;
  } else {
    std::cout << "Usage: ... hive_base name_of_read_bag.bag "
      << "name_of_write_bag.bag" << std::endl;
    return -1;
  }

  std::string read_bag(argv[1]);
  rbag.open(read_bag, rosbag::bagmode::Read);

  // Get current calibration
  if (!ViveUtils::ReadConfig(HIVE_BASE_CALIBRATION_FILE, &cal)) {
    ROS_FATAL("Can't find calibration file.");
    return -1;
  } else {
    ROS_INFO("Read calibration file.");
  }

  // Lighthouses
  rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
  for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
    const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
      bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
    cal.SetLighthouses(*vl);
  }
  ROS_INFO("Lighthouses' setup complete.");

  // Trackers
  rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
  for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
    const hive::ViveCalibrationTrackerArray::ConstPtr vt =
      bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
    cal.SetTrackers(*vt);
    // TODO set up solver.
  }
  for (auto tr_it = cal.trackers.begin();
    tr_it != cal.trackers.end(); tr_it++) {
    solver[tr_it->first] = BaseSolve(cal.environment,
      tr_it->second);
  }
  ROS_INFO("Trackers' setup complete.");

  // Light data
  rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    solver[vl->header.frame_id].ProcessLight(vl);
    geometry_msgs::TransformStamped msg;
    // Save the computed poses
    if (write_bag &&
      solver[vl->header.frame_id].GetTransform(msg)) {
      wbag.write("/tf", ros::Time::now(), msg);
    }
  }

  TFVector tfs = ViveUtils::GetTransforms(cal);
  for (auto tf_it = tfs.begin(); tf_it != tfs.end(); tf_it++) {
    wbag.write("/tf_static", ros::Time::now(), *tf_it);
  }

  rbag.close();
  if (write_bag)
    wbag.close();

  ros::shutdown();
  return 0;
}