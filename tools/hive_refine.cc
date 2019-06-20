// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <hive/vive_refine.h>

// This is a test function
int main(int argc, char ** argv)
{
  // ros intializations
  rosbag::View view;
  rosbag::Bag rbag;

  // Refinery initializations
  Calibration cal;

  // Read the bag name
  if (argc < 2) {
    std::cout << "Usage: ... hive_offset name_of_read_bag.bag" << std::endl;
    return -1;
  }

  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &cal)) {
    ROS_FATAL("Can't find calibration file.");
    return -1;
  } else {
    ROS_INFO("Read calibration file.");
  }

  rbag.open(argv[1], rosbag::bagmode::Read);
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
  }
  ROS_INFO("Trackers' setup complete.");

  size_t counter = 0;
  // Refinery ref = Refinery(cal, true, 1.0e-2, false); // Best static
  // Refinery ref = Refinery(cal, true, 1.0e1, true);
  Refinery ref = Refinery(cal, true, 1.0e1, true);
  // Light data
  std::vector<std::string> topics;
  topics.push_back("/loc/vive/imu");
  topics.push_back("/loc/vive/light");
  topics.push_back("/loc/vive/imu/");
  topics.push_back("/loc/vive/light/");
  rosbag::View view_li(rbag, rosbag::TopicQuery(topics));
  for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
    const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
    if (vl != NULL) {
      counter++;
      // if (counter < 800) continue;
      // if (counter >= 1200) break;
      ref.AddLight(vl);

    }
    const sensor_msgs::Imu::ConstPtr vi = bag_it->instantiate<sensor_msgs::Imu>();
    if (vi != NULL) {
      // if (counter < 800) continue;
      // if (counter >= 20) break;
      ref.AddImu(vi);
    }
  }
  ROS_INFO("Data processment complete.");

  // Solve the refinement
  ref.Solve();

  ViveUtils::WriteConfig(HIVE_CALIBRATION_FILE,
    ref.GetCalibration());

  return 0;
}