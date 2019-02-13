# Message with solved pose for a certain lighthouse and corresponding data to that pose

Header header                         # Metadata
geometry_msgs/Vector3 position        # Position of the tracker
geometry_msgs/Vector3 axisangle       # Axis angle of the tracker

hive/ViveLightSample[] samples
string lighthouse
string tracker
uint8 axis
uint8 AXIS_HORIZONTAL = 0
uint8 AXIS_VERTICAL   = 1
