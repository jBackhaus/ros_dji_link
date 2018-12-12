# A message containing Image and GPS Info

# header.stamp is the ROS Time when the sentence was read.
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header

# This is the original saved image name
std_msgs/String imagename

# This contains the std_msgs/Image message
sensor_msgs/CompressedImage imagedata

# This contains the sensor_msgs/NavSatFix message
sensor_msgs/NavSatFix gpsdata

# Heading in Degree
std_msgs/Float32 heading

# Barometric Height in Meter
std_msgs/Float32 baroHeight

# Pose of the camera
geometry_msgs/Pose pose

# Mission state (on or off)
std_msgs/Int16 missioncount



