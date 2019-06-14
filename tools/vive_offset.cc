/*
 * @version 1.0
 * @author Miguel Rego Borges
 * Instituto Superior Tecnico - University of Lisbon
 * Purpose: calculate offset between vive's frame and optitrack's
*/

#include <hive/hive_offset.h>
#include <hive/hive_solver.h>

int main(int argc, char ** argv) {
  // ros::init(argc, argv, "hive_offset");
  // ros::NodeHandle nh;

  HiveOffset * hiver;

  Calibration calibration;
  std::map<std::string, Solver*> solver;
  rosbag::Bag wbag;
  wbag.open("hive_offset.bag", rosbag::bagmode::Write);


  // Calibration
  if (!ViveUtils::ReadConfig(HIVE_CALIBRATION_FILE, &calibration)) {
    ROS_FATAL("Can't find calibration file.");
    return false;
  } else {
    ROS_INFO("Read calibration file.");
  }

  // No step
  if (argc == 2) {
    ROS_INFO ("NO STEP OFFSET.");

    rosbag::View view;
    rosbag::Bag rbag;
    hiver = new HiveOffset(1.0, true, false);
    rbag.open(argv[1], rosbag::bagmode::Read);

    // Read OptiTrack poses
    rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
    for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
      const tf2_msgs::TFMessage::ConstPtr tf =
        bag_it->instantiate<tf2_msgs::TFMessage>();
      for (auto tf_it = tf->transforms.begin();
        tf_it != tf->transforms.end(); tf_it++) {
        hiver->AddOptiTrackPose(*tf_it);
      }
    }
    ROS_INFO("OptiTrack' setup complete.");

    // Lighthouses
    rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
    for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
      const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
        bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
      calibration.SetLighthouses(*vl);
    }
    ROS_INFO("Lighthouses' setup complete.");

    // Trackers
    rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
    for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
      const hive::ViveCalibrationTrackerArray::ConstPtr vt =
        bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
      calibration.SetTrackers(*vt);
      for (auto tracker : calibration.trackers) {
        solver[tracker.first] = new HiveSolver(tracker.second,
          calibration.lighthouses,
          calibration.environment,
          true);
      }
    }
    ROS_INFO("Trackers' setup complete.");

    // Light data
    size_t counter = 0;
    rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
    for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
      const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
      solver[vl->header.frame_id]->ProcessLight(vl);
      geometry_msgs::TransformStamped msg;
      if (solver[vl->header.frame_id]->GetTransform(msg)) {
        msg.header.stamp = vl->header.stamp;
        hiver->AddVivePose(msg);
        counter++;
      }
      // if (counter >= 400) break;
    }
    rbag.close();
    TFs offsets;
    ROS_INFO("Getting Offset");
    hiver->GetOffset(offsets);

    for (auto of_it = offsets.begin();
      of_it != offsets.end(); of_it++) {
      wbag.write("/offset", ros::TIME_MIN,*of_it);
    }

  // Multiple bags - with step
  } else if (argc > 2) {
    ROS_INFO ("STEP OFFSET.");

    hiver = new HiveOffset(1.0, true, true);

    // Iterate over all bags
    size_t counter;
    for (size_t arg = 1; arg < argc; arg++) {
      rosbag::View view;
      rosbag::Bag rbag;
      rbag.open(argv[arg], rosbag::bagmode::Read);

      // Read OptiTrack poses
      counter = 0;
      rosbag::View view_ot(rbag, rosbag::TopicQuery("/tf"));
      for (auto bag_it = view_ot.begin(); bag_it != view_ot.end(); bag_it++) {
        const tf2_msgs::TFMessage::ConstPtr tf =
          bag_it->instantiate<tf2_msgs::TFMessage>();
        for (auto tf_it = tf->transforms.begin();
          tf_it != tf->transforms.end(); tf_it++) {
          std::cout << "OptiTrack: " <<
            tf_it->transform.translation.x << ", " <<
            tf_it->transform.translation.y << ", " <<
            tf_it->transform.translation.z << ", " <<
            tf_it->transform.rotation.w << ", " <<
            tf_it->transform.rotation.x << ", " <<
            tf_it->transform.rotation.y << ", " <<
            tf_it->transform.rotation.z << std::endl;
          hiver->AddOptiTrackPose(*tf_it);
          counter++;
        }
        if (counter >= 10) {
          break;
        }
      }
      ROS_INFO("OptiTrack' setup complete.");

      // Lighthouses
      rosbag::View view_lh(rbag, rosbag::TopicQuery("/loc/vive/lighthouses"));
      for (auto bag_it = view_lh.begin(); bag_it != view_lh.end(); bag_it++) {
        const hive::ViveCalibrationLighthouseArray::ConstPtr vl =
          bag_it->instantiate<hive::ViveCalibrationLighthouseArray>();
        calibration.SetLighthouses(*vl);
      }
      ROS_INFO("Lighthouses' setup complete.");

      // Trackers
      rosbag::View view_tr(rbag, rosbag::TopicQuery("/loc/vive/trackers"));
      for (auto bag_it = view_tr.begin(); bag_it != view_tr.end(); bag_it++) {
        const hive::ViveCalibrationTrackerArray::ConstPtr vt =
          bag_it->instantiate<hive::ViveCalibrationTrackerArray>();
        calibration.SetTrackers(*vt);
        for (auto tracker : calibration.trackers) {
          solver[tracker.first] = new HiveSolver(tracker.second,
            calibration.lighthouses,
            calibration.environment,
            true);
        }
      }
      ROS_INFO("Trackers' setup complete.");

      // Light data
      counter = 0;
      size_t unsuccess = 0;
      rosbag::View view_li(rbag, rosbag::TopicQuery("/loc/vive/light"));
      for (auto bag_it = view_li.begin(); bag_it != view_li.end(); bag_it++) {
        const hive::ViveLight::ConstPtr vl = bag_it->instantiate<hive::ViveLight>();
        solver[vl->header.frame_id]->ProcessLight(vl);
        // usleep(50000);
        // ROS_INFO("OI");
        geometry_msgs::TransformStamped msg;
        if (solver[vl->header.frame_id]->GetTransform(msg)) {
          msg.header.stamp = vl->header.stamp;
          hiver->AddVivePose(msg);
          std::cout << "Vive: " <<
            msg.transform.translation.x << ", " <<
            msg.transform.translation.y << ", " <<
            msg.transform.translation.z << ", " <<
            msg.transform.rotation.w << ", " <<
            msg.transform.rotation.x << ", " <<
            msg.transform.rotation.y << ", " <<
            msg.transform.rotation.z << std::endl;
          counter++;
        } else {
          unsuccess++;
        }
        if (counter >= 20 || unsuccess >= 100) {
          break;
        }
      }
      hiver->NextPose();
      rbag.close();
    }
    TFs offsets;
    ROS_INFO("OFFSET");
    hiver->GetOffset(offsets);

    for (auto of_it = offsets.begin();
      of_it != offsets.end(); of_it++) {
      wbag.write("/offset", ros::TIME_MIN,*of_it);
    }

  } else {
    TFs vive, optitrack, offsets;
    TestTrajectory(vive, optitrack);
    hiver = new HiveOffset(1.0, true, true);

    for (size_t i = 0; i < 10; i++) {
      hiver->AddVivePose(vive[i]);
      hiver->AddOptiTrackPose(optitrack[i]);
      hiver->NextPose();
    }

    hiver->GetOffset(offsets);

  }

  return 0;
}

