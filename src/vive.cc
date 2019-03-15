/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// ROS includes
#include <hive/vive.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

bool ViveUtils::WriteConfig(std::string file_name, Calibration const& calibration) {
  std::ofstream ofs(file_name.c_str(), std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    ROS_INFO_STREAM("Cannot write to file " << file_name);
    return false;
  }

  hive::ViveCalibration vive_calibration;

  // Environment
  vive_calibration.calibration.translation.x = calibration.environment.vive.translation.x;
  vive_calibration.calibration.translation.y = calibration.environment.vive.translation.y;
  vive_calibration.calibration.translation.z = calibration.environment.vive.translation.z;
  vive_calibration.calibration.rotation.w = calibration.environment.vive.rotation.w;
  vive_calibration.calibration.rotation.x = calibration.environment.vive.rotation.x;
  vive_calibration.calibration.rotation.y = calibration.environment.vive.rotation.y;
  vive_calibration.calibration.rotation.z = calibration.environment.vive.rotation.z;

  // Lighthouses
  for (std::map<std::string, Transform>::const_iterator lh_it = calibration.environment.lighthouses.begin();
    lh_it != calibration.environment.lighthouses.end(); lh_it++) {
    geometry_msgs::TransformStamped lighthouse;
    lighthouse.header.frame_id = lh_it->second.parent_frame;
    lighthouse.child_frame_id = lh_it->second.child_frame;
    lighthouse.transform.translation.x = lh_it->second.translation.x;
    lighthouse.transform.translation.y = lh_it->second.translation.y;
    lighthouse.transform.translation.z = lh_it->second.translation.z;
    lighthouse.transform.rotation.w = lh_it->second.rotation.w;
    lighthouse.transform.rotation.x = lh_it->second.rotation.x;
    lighthouse.transform.rotation.y = lh_it->second.rotation.y;
    lighthouse.transform.rotation.z = lh_it->second.rotation.z;
    vive_calibration.lighthouse.push_back(lighthouse);
  }

  uint32_t serial_size = ros::serialization::serializationLength(vive_calibration);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, vive_calibration);
  ofs.write(reinterpret_cast<char*>(obuffer.get()), serial_size);
  ofs.close();
  return true;
}

bool ViveUtils::ReadConfig(std::string file_name, Calibration * calibration) {
  std::ifstream ifs(file_name.c_str(), std::ios::in | std::ios::binary);
  hive::ViveCalibration vive_calibration;
  if (!ifs.good()) {
    ROS_INFO_STREAM("Cannot read from file " << file_name);
    return false;
  }
  // Read data
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();
  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read(reinterpret_cast<char*>(ibuffer.get()), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, vive_calibration);
  ifs.close();

  // Converting

  // Environment
  // Vive
  (*calibration).environment.vive.parent_frame = "world";
  (*calibration).environment.vive.child_frame = "vive";
  (*calibration).environment.vive.translation.x = vive_calibration.calibration.translation.x;
  (*calibration).environment.vive.translation.y = vive_calibration.calibration.translation.y;
  (*calibration).environment.vive.translation.z = vive_calibration.calibration.translation.z;
  (*calibration).environment.vive.rotation.w = vive_calibration.calibration.rotation.w;
  (*calibration).environment.vive.rotation.x = vive_calibration.calibration.rotation.x;
  (*calibration).environment.vive.rotation.y = vive_calibration.calibration.rotation.y;
  (*calibration).environment.vive.rotation.z = vive_calibration.calibration.rotation.z;
  // Lighthouses
  for (std::vector<geometry_msgs::TransformStamped>::iterator lh_it = vive_calibration.lighthouse.begin();
    lh_it != vive_calibration.lighthouse.end(); lh_it++) {
    Transform lighthouse;
    lighthouse.parent_frame = lh_it->header.frame_id;
    lighthouse.child_frame = lh_it->child_frame_id;
    lighthouse.translation.x = lh_it->transform.translation.x;
    lighthouse.translation.y = lh_it->transform.translation.y;
    lighthouse.translation.z = lh_it->transform.translation.z;
    lighthouse.rotation.w = lh_it->transform.rotation.w;
    lighthouse.rotation.x = lh_it->transform.rotation.x;
    lighthouse.rotation.y = lh_it->transform.rotation.y;
    lighthouse.rotation.z = lh_it->transform.rotation.z;
    (*calibration).environment.lighthouses[lh_it->child_frame_id] = lighthouse;
  }

  return true;
}

bool ViveUtils::SendTransforms(Calibration const& calibration_data) {
  // Broadcaster of all the trasnforms read from the config file
  static tf2_ros::StaticTransformBroadcaster static_br;
  geometry_msgs::TransformStamped world_tf;
  world_tf.header.stamp = ros::Time::now();
  world_tf.header.frame_id = calibration_data.environment.vive.parent_frame;
  world_tf.child_frame_id = calibration_data.environment.vive.child_frame;
  world_tf.transform.translation.x = calibration_data.environment.vive.translation.x;
  world_tf.transform.translation.y = calibration_data.environment.vive.translation.y;
  world_tf.transform.translation.z = calibration_data.environment.vive.translation.z;
  world_tf.transform.rotation.w = calibration_data.environment.vive.rotation.w;
  world_tf.transform.rotation.x = calibration_data.environment.vive.rotation.x;
  world_tf.transform.rotation.y = calibration_data.environment.vive.rotation.y;
  world_tf.transform.rotation.z = calibration_data.environment.vive.rotation.z;
  static_br.sendTransform(world_tf);

  for (std::map<std::string, Transform>::const_iterator lh_it = calibration_data.environment.lighthouses.begin();
    lh_it != calibration_data.environment.lighthouses.end(); lh_it++) {
    geometry_msgs::TransformStamped lighthouse_tf;
    lighthouse_tf.header.stamp = ros::Time::now();
    lighthouse_tf.header.frame_id = lh_it->second.parent_frame;
    lighthouse_tf.child_frame_id = lh_it->second.child_frame;
    lighthouse_tf.transform.translation.x = lh_it->second.translation.x;
    lighthouse_tf.transform.translation.y = lh_it->second.translation.y;
    lighthouse_tf.transform.translation.z = lh_it->second.translation.z;
    lighthouse_tf.transform.rotation.w = lh_it->second.rotation.w;
    lighthouse_tf.transform.rotation.x = lh_it->second.rotation.x;
    lighthouse_tf.transform.rotation.y = lh_it->second.rotation.y;
    lighthouse_tf.transform.rotation.z = lh_it->second.rotation.z;
    static_br.sendTransform(lighthouse_tf);
  }
  return true;
}

size_t ViveUtils::ConvertExtrinsics(Tracker const& tracker, double * extrinsics) {
  for (std::map<uint8_t, Sensor>::const_iterator sn_it = tracker.sensors.begin();
    sn_it != tracker.sensors.end(); sn_it++) {
    if (unsigned(sn_it->first) >= TRACKER_SENSORS_NUMBER) {
      return -1;
    }
    extrinsics[3 * unsigned(sn_it->first)] =     sn_it->second.position.x;
    extrinsics[3 * unsigned(sn_it->first) + 1] = sn_it->second.position.y;
    extrinsics[3 * unsigned(sn_it->first) + 2] = sn_it->second.position.z;
  }
  return tracker.sensors.size();
}

bool Calibration::SetEnvironment(hive::ViveCalibration const& msg) {
  environment.vive.parent_frame = "world";
  environment.vive.child_frame = "vive";
  environment.vive.translation.x = msg.calibration.translation.x;
  environment.vive.translation.y = msg.calibration.translation.y;
  environment.vive.translation.z = msg.calibration.translation.z;
  environment.vive.rotation.w = msg.calibration.rotation.w;
  environment.vive.rotation.x = msg.calibration.rotation.x;
  environment.vive.rotation.y = msg.calibration.rotation.y;
  environment.vive.rotation.z = msg.calibration.rotation.z;

  // Lighthouse
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator lh_it = msg.lighthouse.begin();
    lh_it != msg.lighthouse.end(); lh_it++) {
    environment.lighthouses[lh_it->child_frame_id].parent_frame = lh_it->header.frame_id;
    environment.lighthouses[lh_it->child_frame_id].child_frame = lh_it->child_frame_id;

    environment.lighthouses[lh_it->child_frame_id].translation.x = lh_it->transform.translation.x;
    environment.lighthouses[lh_it->child_frame_id].translation.y = lh_it->transform.translation.y;
    environment.lighthouses[lh_it->child_frame_id].translation.z = lh_it->transform.translation.z;

    environment.lighthouses[lh_it->child_frame_id].rotation.w = lh_it->transform.rotation.w;
    environment.lighthouses[lh_it->child_frame_id].rotation.x = lh_it->transform.rotation.x;
    environment.lighthouses[lh_it->child_frame_id].rotation.y = lh_it->transform.rotation.y;
    environment.lighthouses[lh_it->child_frame_id].rotation.z = lh_it->transform.rotation.z;
  }

  // Body
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator bd_it = msg.body.begin();
    bd_it != msg.body.end(); bd_it++) {
    environment.bodies[bd_it->header.frame_id].parent_frame = bd_it->header.frame_id;
    environment.bodies[bd_it->header.frame_id].child_frame = bd_it->child_frame_id;

    environment.bodies[bd_it->header.frame_id].translation.x = bd_it->transform.translation.x;
    environment.bodies[bd_it->header.frame_id].translation.y = bd_it->transform.translation.y;
    environment.bodies[bd_it->header.frame_id].translation.z = bd_it->transform.translation.z;

    environment.bodies[bd_it->header.frame_id].rotation.w = bd_it->transform.rotation.w;
    environment.bodies[bd_it->header.frame_id].rotation.x = bd_it->transform.rotation.x;
    environment.bodies[bd_it->header.frame_id].rotation.y = bd_it->transform.rotation.y;
    environment.bodies[bd_it->header.frame_id].rotation.z = bd_it->transform.rotation.z;
  }
  return true;
}

bool Calibration::GetEnvironment(hive::ViveCalibration * msg) {
  (*msg).calibration.translation.x = environment.vive.translation.x;
  (*msg).calibration.translation.y = environment.vive.translation.y;
  (*msg).calibration.translation.z = environment.vive.translation.z;
  (*msg).calibration.rotation.w = environment.vive.rotation.w;
  (*msg).calibration.rotation.x = environment.vive.rotation.x;
  (*msg).calibration.rotation.y = environment.vive.rotation.y;
  (*msg).calibration.rotation.z = environment.vive.rotation.z;

  // Lighthouse
  for (std::map<std::string, Transform>::iterator lh_it = environment.lighthouses.begin();
    lh_it != environment.lighthouses.end(); lh_it++) {
    geometry_msgs::TransformStamped lh_msg;
    lh_msg.header.frame_id = lh_it->second.parent_frame;
    lh_msg.child_frame_id = lh_it->second.child_frame;

    lh_msg.transform.translation.x = lh_it->second.translation.x;
    lh_msg.transform.translation.y = lh_it->second.translation.y;
    lh_msg.transform.translation.z = lh_it->second.translation.z;

    lh_msg.transform.rotation.w = lh_it->second.rotation.w;
    lh_msg.transform.rotation.x = lh_it->second.rotation.x;
    lh_msg.transform.rotation.y = lh_it->second.rotation.y;
    lh_msg.transform.rotation.z = lh_it->second.rotation.z;
    (*msg).lighthouse.push_back(lh_msg);
  }

  // Body
  for (std::map<std::string, Transform>::iterator bd_it = environment.bodies.begin();
    bd_it != environment.bodies.end(); bd_it++) {
    geometry_msgs::TransformStamped bd_msg;
    bd_msg.header.frame_id = bd_it->second.parent_frame;
    bd_msg.child_frame_id = bd_it->second.child_frame;

    bd_msg.transform.translation.x = bd_it->second.translation.x;
    bd_msg.transform.translation.y = bd_it->second.translation.y;
    bd_msg.transform.translation.z = bd_it->second.translation.z;

    bd_msg.transform.rotation.w = bd_it->second.rotation.w;
    bd_msg.transform.rotation.x = bd_it->second.rotation.x;
    bd_msg.transform.rotation.y = bd_it->second.rotation.y;
    bd_msg.transform.rotation.z = bd_it->second.rotation.z;
    (*msg).body.push_back(bd_msg);
  }
  return true;
}

bool Calibration::SetLightSpecs(hive::ViveCalibrationGeneral const& msg) {
  light_specs.timebase_hz = msg.timebase_hz;
  light_specs.timecenter_ticks = msg.timecenter_ticks;
  light_specs.pulsedist_max_ticks = msg.pulsedist_max_ticks;
  light_specs.pulselength_min_sync = msg.pulselength_min_sync;
  light_specs.pulse_in_clear_time = msg.pulse_in_clear_time;
  light_specs.pulse_max_for_sweep = msg.pulse_max_for_sweep;
  light_specs.pulse_synctime_offset = msg.pulse_synctime_offset;
  light_specs.pulse_synctime_slack = msg.pulse_synctime_slack;
  return true;
}

bool Calibration::GetLightSpecs(hive::ViveCalibrationGeneral * msg) {
  (*msg).timebase_hz = light_specs.timebase_hz;
  (*msg).timecenter_ticks = light_specs.timecenter_ticks;
  (*msg).pulsedist_max_ticks = light_specs.pulsedist_max_ticks;
  (*msg).pulselength_min_sync = light_specs.pulselength_min_sync;
  (*msg).pulse_in_clear_time = light_specs.pulse_in_clear_time;
  (*msg).pulse_max_for_sweep = light_specs.pulse_max_for_sweep;
  (*msg).pulse_synctime_offset = light_specs.pulse_synctime_offset;
  (*msg).pulse_synctime_slack = light_specs.pulse_synctime_slack;
  return true;
}

bool Calibration::SetLighthouses(hive::ViveCalibrationLighthouseArray const& msg) {
  for (std::vector<hive::ViveCalibrationLighthouse>::const_iterator lh_it = msg.lighthouses.begin();
    lh_it != msg.lighthouses.end(); lh_it++) {
    lighthouses[lh_it->serial].serial = lh_it->serial;
    lighthouses[lh_it->serial].id = lh_it->id;
    // Vertical Motor
    lighthouses[lh_it->serial].vertical_motor.phase = lh_it->vertical.phase;
    lighthouses[lh_it->serial].vertical_motor.tilt = lh_it->vertical.tilt;
    lighthouses[lh_it->serial].vertical_motor.gib_phase = lh_it->vertical.gibphase;
    lighthouses[lh_it->serial].vertical_motor.gib_magnitude = lh_it->vertical.gibmag;
    lighthouses[lh_it->serial].vertical_motor.curve = lh_it->vertical.curve;
    // Horizontal Motor
    lighthouses[lh_it->serial].horizontal_motor.phase = lh_it->horizontal.phase;
    lighthouses[lh_it->serial].horizontal_motor.tilt = lh_it->horizontal.tilt;
    lighthouses[lh_it->serial].horizontal_motor.gib_phase = lh_it->horizontal.gibphase;
    lighthouses[lh_it->serial].horizontal_motor.gib_magnitude = lh_it->horizontal.gibmag;
    lighthouses[lh_it->serial].horizontal_motor.curve = lh_it->horizontal.curve;
  }
  return true;
}

bool Calibration::GetLighthouses(hive::ViveCalibrationLighthouseArray * msg) {
  for (std::map<std::string, Lighthouse>::iterator lh_it = lighthouses.begin();
    lh_it != lighthouses.end(); lh_it++) {
    hive::ViveCalibrationLighthouse lh_msg;
    lh_msg.serial = lh_it->second.serial;
    lh_msg.id = lh_it->second.id;
    // Vertical Motor
    lh_msg.vertical.phase = lh_it->second.vertical_motor.phase;
    lh_msg.vertical.tilt = lh_it->second.vertical_motor.tilt;
    lh_msg.vertical.gibphase = lh_it->second.vertical_motor.gib_phase;
    lh_msg.vertical.gibmag = lh_it->second.vertical_motor.gib_magnitude;
    lh_msg.vertical.curve = lh_it->second.vertical_motor.curve;
    // Horizontal Motor
    lh_msg.horizontal.phase = lh_it->second.horizontal_motor.phase;
    lh_msg.horizontal.tilt = lh_it->second.horizontal_motor.tilt;
    lh_msg.horizontal.gibphase = lh_it->second.horizontal_motor.gib_phase;
    lh_msg.horizontal.gibmag = lh_it->second.horizontal_motor.gib_magnitude;
    lh_msg.horizontal.curve = lh_it->second.horizontal_motor.curve;
    (*msg).lighthouses.push_back(lh_msg);
  }
  return true;
}

bool Calibration::SetTrackers(hive::ViveCalibrationTrackerArray const& msg) {
  for (std::vector<hive::ViveCalibrationTracker>::const_iterator tr_it = msg.trackers.begin();
    tr_it != msg.trackers.end(); tr_it++) {
    // trackers[tr_it->serial].frame = tr_it->serial;
    trackers[tr_it->serial].serial = tr_it->serial;

    // Acc bias
    trackers[tr_it->serial].acc_bias.x = tr_it->acc_bias.x;
    trackers[tr_it->serial].acc_bias.y = tr_it->acc_bias.y;
    trackers[tr_it->serial].acc_bias.z = tr_it->acc_bias.z;

    // Acc scale
    trackers[tr_it->serial].acc_scale.x = tr_it->acc_scale.x;
    trackers[tr_it->serial].acc_scale.y = tr_it->acc_scale.y;
    trackers[tr_it->serial].acc_scale.z = tr_it->acc_scale.z;

    // Gyr bias
    trackers[tr_it->serial].gyr_bias.x = tr_it->gyr_bias.x;
    trackers[tr_it->serial].gyr_bias.y = tr_it->gyr_bias.y;
    trackers[tr_it->serial].gyr_bias.z = tr_it->gyr_bias.z;

    // Gyr scale
    trackers[tr_it->serial].gyr_scale.x = tr_it->gyr_scale.x;
    trackers[tr_it->serial].gyr_scale.y = tr_it->gyr_scale.y;
    trackers[tr_it->serial].gyr_scale.z = tr_it->gyr_scale.z;

    for (std::vector<hive::ViveExtrinsics>::const_iterator ss_it = tr_it->extrinsics.begin();
      ss_it != tr_it->extrinsics.end(); ss_it++) {
      // Sensor positions
      trackers[tr_it->serial].sensors[ss_it->id].position.x = ss_it->position.x;
      trackers[tr_it->serial].sensors[ss_it->id].position.y = ss_it->position.y;
      trackers[tr_it->serial].sensors[ss_it->id].position.z = ss_it->position.z;
      // Sensors normal
      trackers[tr_it->serial].sensors[ss_it->id].normal.x = ss_it->normal.x;
      trackers[tr_it->serial].sensors[ss_it->id].normal.y = ss_it->normal.y;
      trackers[tr_it->serial].sensors[ss_it->id].normal.z = ss_it->normal.z;
    }
  }

  return true;
}

bool Calibration::GetTrackers(hive::ViveCalibrationTrackerArray * msg) {
  for (std::map<std::string, Tracker>::iterator tr_it = trackers.begin();
    tr_it != trackers.end(); tr_it++) {
    hive::ViveCalibrationTracker tracker_msg;
    tracker_msg.serial = tr_it->first;

    // Acc bias
    tracker_msg.acc_bias.x = tr_it->second.acc_bias.x;
    tracker_msg.acc_bias.y = tr_it->second.acc_bias.y;
    tracker_msg.acc_bias.z = tr_it->second.acc_bias.z;

    // Acc scale
    tracker_msg.acc_scale.x = tr_it->second.acc_scale.x;
    tracker_msg.acc_scale.y = tr_it->second.acc_scale.y;
    tracker_msg.acc_scale.z = tr_it->second.acc_scale.z;

    // Gyr bias
    tracker_msg.gyr_bias.x = tr_it->second.gyr_bias.x;
    tracker_msg.gyr_bias.y = tr_it->second.gyr_bias.y;
    tracker_msg.gyr_bias.z = tr_it->second.gyr_bias.z;

    // Gyr scale
    tracker_msg.gyr_scale.x = tr_it->second.gyr_scale.x;
    tracker_msg.gyr_scale.y = tr_it->second.gyr_scale.y;
    tracker_msg.gyr_scale.z = tr_it->second.gyr_scale.z;

    for (std::map<uint8_t, Sensor>::iterator ss_it = tr_it->second.sensors.begin();
      ss_it != tr_it->second.sensors.end(); ss_it++) {
      hive::ViveExtrinsics sensor_msg;
      sensor_msg.id = ss_it->first;
      // Sensor positions
      sensor_msg.position.x = ss_it->second.position.x;
      sensor_msg.position.y = ss_it->second.position.y;
      sensor_msg.position.z = ss_it->second.position.z;
      // Sensor normal
      sensor_msg.normal.x = ss_it->second.normal.x;
      sensor_msg.normal.y = ss_it->second.normal.y;
      sensor_msg.normal.z = ss_it->second.normal.z;
      tracker_msg.extrinsics.push_back(sensor_msg);
    }
    (*msg).trackers.push_back(tracker_msg);
  }
  return true;
}

JsonParser::JsonParser(const char * filename) {
  FILE * fp = fopen(filename, "r");
  char readBuffer[65536];
  rapidjson::FileReadStream frs(fp, readBuffer, sizeof(readBuffer));
  document_ = new rapidjson::Document();
  document_->ParseStream(frs);
  fclose(fp);
  if (document_->HasParseError()) {
    std::cout << "ParseError" << std::endl;
  }
}

JsonParser::~JsonParser() {
  delete document_;
}

double JsonParser::GetRate() {
  if (document_->HasMember("rate") && ((*document_)["rate"].IsDouble() || (*document_)["rate"].IsInt())) {
    return (*document_)["rate"].GetDouble();
  }
  return 10.0;
}

bool JsonParser::GetCalibration(Calibration * calibration) {
  // Lighthouses
  if (document_->HasMember("lighthouses") && (*document_)["lighthouses"].IsArray()) {
    for (rapidjson::SizeType i = 0; i < (*document_)["lighthouses"].Size(); i++) {
      std::string lh_child = (*document_)["lighthouses"][i]["frame"].GetString();
      std::string lh_parent = (*document_)["lighthouses"][i]["parent"].GetString();
      Transform lh_transform;
      lh_transform.child_frame = lh_child;
      lh_transform.parent_frame = lh_parent;
      lh_transform.translation.x = (*document_)["lighthouses"][i]["t"]["x"].GetDouble();
      lh_transform.translation.y = (*document_)["lighthouses"][i]["t"]["y"].GetDouble();
      lh_transform.translation.z = (*document_)["lighthouses"][i]["t"]["z"].GetDouble();
      lh_transform.rotation.w = (*document_)["lighthouses"][i]["r"]["w"].GetDouble();
      lh_transform.rotation.x = (*document_)["lighthouses"][i]["r"]["x"].GetDouble();
      lh_transform.rotation.y = (*document_)["lighthouses"][i]["r"]["y"].GetDouble();
      lh_transform.rotation.z = (*document_)["lighthouses"][i]["r"]["w"].GetDouble();
      // Normalization of the quaternion to make it valid
      double norm = sqrt(lh_transform.rotation.w * lh_transform.rotation.w +
        lh_transform.rotation.x + lh_transform.rotation.x +
        lh_transform.rotation.y + lh_transform.rotation.y +
        lh_transform.rotation.z + lh_transform.rotation.z);
      lh_transform.rotation.w = lh_transform.rotation.w / norm;
      lh_transform.rotation.x = lh_transform.rotation.x / norm;
      lh_transform.rotation.y = lh_transform.rotation.y / norm;
      lh_transform.rotation.z = lh_transform.rotation.z / norm;
      // Save the transforms
      calibration->environment.lighthouses[lh_child] = lh_transform;
    }
  }
  // Body
  if (document_->HasMember("body")) {
    if (document_->HasMember("frame") && (*document_)["frame"].IsString()) {
      std::string body_name = (*document_)["frame"].GetString();
      calibration->environment.offset.child_frame = body_name;
      for (rapidjson::SizeType i = 0; i < (*document_)["body"]["parents"].Size(); i++) {
        std::string device_name = (*document_)["body"]["parents"][i]["frame"].GetString();
        Transform body_transform;
        body_transform.child_frame = body_name;
        body_transform.parent_frame = device_name;
        body_transform.translation.x = (*document_)["body"]["parents"][i]["t"]["x"].GetDouble();
        body_transform.translation.y = (*document_)["body"]["parents"][i]["t"]["y"].GetDouble();
        body_transform.translation.z = (*document_)["body"]["parents"][i]["t"]["z"].GetDouble();
        body_transform.rotation.w = (*document_)["body"]["parents"][i]["r"]["w"].GetDouble();
        body_transform.rotation.x = (*document_)["body"]["parents"][i]["r"]["x"].GetDouble();
        body_transform.rotation.y = (*document_)["body"]["parents"][i]["r"]["y"].GetDouble();
        body_transform.rotation.z = (*document_)["body"]["parents"][i]["r"]["z"].GetDouble();
        // Normalization of the quaternion to make it valid
        double norm = sqrt(body_transform.rotation.w * body_transform.rotation.w +
          body_transform.rotation.x + body_transform.rotation.x +
          body_transform.rotation.y + body_transform.rotation.y +
          body_transform.rotation.z + body_transform.rotation.z);
        body_transform.rotation.w = body_transform.rotation.w / norm;
        body_transform.rotation.x = body_transform.rotation.x / norm;
        body_transform.rotation.y = body_transform.rotation.y / norm;
        body_transform.rotation.z = body_transform.rotation.z / norm;
        // Save the transforms
        calibration->environment.bodies[device_name] = body_transform;
      }
    }
  }
  // Vive
  if (document_->HasMember("vive")) {
    calibration->environment.vive.child_frame = "vive";
    calibration->environment.vive.parent_frame = "world";
    calibration->environment.vive.translation.x = (*document_)["vive"]["t"]["x"].GetDouble();
    calibration->environment.vive.translation.y = (*document_)["vive"]["t"]["y"].GetDouble();
    calibration->environment.vive.translation.z = (*document_)["vive"]["t"]["z"].GetDouble();
    calibration->environment.vive.rotation.w = (*document_)["vive"]["r"]["w"].GetDouble();
    calibration->environment.vive.rotation.x = (*document_)["vive"]["r"]["x"].GetDouble();
    calibration->environment.vive.rotation.y = (*document_)["vive"]["r"]["y"].GetDouble();
    calibration->environment.vive.rotation.z = (*document_)["vive"]["r"]["z"].GetDouble();
    // Normalization of the quaternion to make it valid
    double norm = sqrt(calibration->environment.vive.rotation.w * calibration->environment.vive.rotation.w +
      calibration->environment.vive.rotation.x + calibration->environment.vive.rotation.x +
      calibration->environment.vive.rotation.y + calibration->environment.vive.rotation.y +
      calibration->environment.vive.rotation.z + calibration->environment.vive.rotation.z);
    calibration->environment.vive.rotation.w = calibration->environment.vive.rotation.w / norm;
    calibration->environment.vive.rotation.x = calibration->environment.vive.rotation.x / norm;
    calibration->environment.vive.rotation.y = calibration->environment.vive.rotation.y / norm;
    calibration->environment.vive.rotation.z = calibration->environment.vive.rotation.z / norm;
  }
  // Offset
  if (document_->HasMember("offset")) {
    calibration->environment.offset.child_frame = "vive";
    calibration->environment.offset.parent_frame = "world";
    calibration->environment.offset.translation.x = (*document_)["offset"]["t"]["x"].GetDouble();
    calibration->environment.offset.translation.y = (*document_)["offset"]["t"]["y"].GetDouble();
    calibration->environment.offset.translation.z = (*document_)["offset"]["t"]["z"].GetDouble();
    calibration->environment.offset.rotation.w = (*document_)["offset"]["r"]["w"].GetDouble();
    calibration->environment.offset.rotation.x = (*document_)["offset"]["r"]["x"].GetDouble();
    calibration->environment.offset.rotation.y = (*document_)["offset"]["r"]["y"].GetDouble();
    calibration->environment.offset.rotation.z = (*document_)["offset"]["r"]["z"].GetDouble();
    // Normalization of the quaternion to make it valid
    double norm = sqrt(calibration->environment.offset.rotation.w * calibration->environment.offset.rotation.w +
      calibration->environment.offset.rotation.x + calibration->environment.offset.rotation.x +
      calibration->environment.offset.rotation.y + calibration->environment.offset.rotation.y +
      calibration->environment.offset.rotation.z + calibration->environment.offset.rotation.z);
    calibration->environment.offset.rotation.w = calibration->environment.offset.rotation.w / norm;
    calibration->environment.offset.rotation.x = calibration->environment.offset.rotation.x / norm;
    calibration->environment.offset.rotation.y = calibration->environment.offset.rotation.y / norm;
    calibration->environment.offset.rotation.z = calibration->environment.offset.rotation.z / norm;
  }
  return true;
}

bool JsonParser::GetBody(Calibration * calibration) {
  // Body
  if (document_->HasMember("body")) {
    std::cout << "body " << std::endl;
    if ((*document_)["body"].HasMember("frame") && (*document_)["body"]["frame"].IsString()) {
      std::cout << "  " << (*document_)["body"]["frame"].GetString() << std::endl;
      std::string body_name = (*document_)["body"]["frame"].GetString();
      calibration->environment.offset.child_frame = body_name;
      for (rapidjson::SizeType i = 0; i < (*document_)["body"]["parents"].Size(); i++) {
        std::string device_name = (*document_)["body"]["parents"][i]["frame"].GetString();
        std::cout << "    " << device_name << std::endl;
        Transform body_transform;
        body_transform.child_frame = body_name;
        body_transform.parent_frame = device_name;
        body_transform.translation.x = (*document_)["body"]["parents"][i]["t"]["x"].GetDouble();
        body_transform.translation.y = (*document_)["body"]["parents"][i]["t"]["y"].GetDouble();
        body_transform.translation.z = (*document_)["body"]["parents"][i]["t"]["z"].GetDouble();
        body_transform.rotation.w = (*document_)["body"]["parents"][i]["r"]["w"].GetDouble();
        body_transform.rotation.x = (*document_)["body"]["parents"][i]["r"]["x"].GetDouble();
        body_transform.rotation.y = (*document_)["body"]["parents"][i]["r"]["y"].GetDouble();
        body_transform.rotation.z = (*document_)["body"]["parents"][i]["r"]["z"].GetDouble();
        // Normalization of the quaternion to make it valid
        double norm = sqrt(body_transform.rotation.w * body_transform.rotation.w +
          body_transform.rotation.x + body_transform.rotation.x +
          body_transform.rotation.y + body_transform.rotation.y +
          body_transform.rotation.z + body_transform.rotation.z);
        body_transform.rotation.w = body_transform.rotation.w / norm;
        body_transform.rotation.x = body_transform.rotation.x / norm;
        body_transform.rotation.y = body_transform.rotation.y / norm;
        body_transform.rotation.z = body_transform.rotation.z / norm;
        // Save the transform in the calibration object
        calibration->environment.bodies[device_name] = body_transform;
      }
    }
  }
  // Offset
  if (document_->HasMember("vive")) {
    std::cout << "vive" << std::endl;
    calibration->environment.offset.child_frame = "vive";
    calibration->environment.offset.parent_frame = "world";
    calibration->environment.offset.translation.x = (*document_)["vive"]["t"]["x"].GetDouble();
    calibration->environment.offset.translation.y = (*document_)["vive"]["t"]["y"].GetDouble();
    calibration->environment.offset.translation.z = (*document_)["vive"]["t"]["z"].GetDouble();
    calibration->environment.offset.rotation.w = (*document_)["vive"]["r"]["w"].GetDouble();
    calibration->environment.offset.rotation.x = (*document_)["vive"]["r"]["x"].GetDouble();
    calibration->environment.offset.rotation.y = (*document_)["vive"]["r"]["y"].GetDouble();
    calibration->environment.offset.rotation.z = (*document_)["vive"]["r"]["z"].GetDouble();
    // Normalization of the quaternion to make it valid
    double norm = sqrt(calibration->environment.offset.rotation.w * calibration->environment.offset.rotation.w +
      calibration->environment.offset.rotation.x + calibration->environment.offset.rotation.x +
      calibration->environment.offset.rotation.y + calibration->environment.offset.rotation.y +
      calibration->environment.offset.rotation.z + calibration->environment.offset.rotation.z);
    calibration->environment.offset.rotation.w = calibration->environment.offset.rotation.w / norm;
    calibration->environment.offset.rotation.x = calibration->environment.offset.rotation.x / norm;
    calibration->environment.offset.rotation.y = calibration->environment.offset.rotation.y / norm;
    calibration->environment.offset.rotation.z = calibration->environment.offset.rotation.z / norm;
  }
  return true;
}

StateMachine::StateMachine() {
  // pass
}

StateMachine::~StateMachine() {
  // pass
}

void StateMachine::SetState(int state) {
  state_ = state;
}

void StateMachine::AddTransition(int from, int action, int to) {
  machine_[from][action] = to;
}

void StateMachine::Update(int action) {
  state_ = machine_[state_][action];
}

int StateMachine::GetState() {
  return state_;
}

void StateMachine::Print() {
  for(std::map<int,std::map<int,int>>::iterator from_it = machine_.begin(); from_it != machine_.end(); from_it++) {
    std::cout << from_it->first;
    for(std::map<int,int>::iterator action_it = from_it->second.begin(); action_it != from_it->second.end(); action_it++) {
      std::cout << " " << action_it->first << "->" << action_it->second;
    }
    std::cout << std::endl;
  }
}


template <typename T>
void hCorrection(T const * position, T const * corrections, T * ang) {
  *ang = atan(position[0]/position[2]);
  return;
}

template <typename T>
void vCorrection(T const * position, T const * corrections, T * ang) {
  *ang = atan(position[1]/position[2]);
  return;
}