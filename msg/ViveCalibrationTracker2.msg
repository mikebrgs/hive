# Get the calibration information for a tracker

string serial                               # Tracker serial number
time timestamp                              # Time of last update
hive/ViveExtrinsics[] extrinsics            # Photodiode extrinsics
geometry_msgs/Vector3 acc_bias              # Acceleromater bias
geometry_msgs/Vector3 acc_scale             # Accelerometer scale
geometry_msgs/Vector3 gyr_bias              # Gyro bias
geometry_msgs/Vector3 gyr_scale             # Gyro scale
geometry_msgs/Transform imu_transform       # IMU frame transform to light frame
geometry_msgs/Transform head_transform      # HEAD frame transform to light frame