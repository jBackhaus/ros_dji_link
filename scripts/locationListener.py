#!/usr/bin/env python

#Created by Jan Backhaus on 12.12.18 as part of the project ros_dji_link.

import sys
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Quaternion
from sensor_msgs.msg import NavSatFix
from gps_common.msg import GPSFix
from std_msgs.msg import Float64, Float32
from visualization_msgs.msg import Marker
import tf



def initDroneMarker():

    drone_marker.header.frame_id="local_origin"
    drone_marker.ns = "dji_link"
    drone_marker.id = 0

    drone_marker.type = 10   #mesh
    drone_marker.mesh_resource = "package://dji_link/meshes/drone.stl"
    drone_marker.action = 0 #add/modify

    drone_marker.scale.x = 20
    drone_marker.scale.y = 20
    drone_marker.scale.z = 20
    #drone_marker.lifetime = 0       #forever

    drone_marker.color.r=1
    drone_marker.color.g=0
    drone_marker.color.b=0
    drone_marker.color.a=1
    

    return


def updateMarkerPose(pose):
    drone_marker.header.stamp = rospy.get_rostime()
    drone_marker.pose = pose
    publishMarker()
    return


def publishPosition():
    position_publisher.publish(position)
    return


def publishMarker():
    marker_publisher.publish(drone_marker)
    return


def updateLocation(location):

    position.latitude = location.latitude
    position.longitude = location.longitude
    position.altitude = location.altitude   
    return


def updateAttitude(odometry):
    quaternion = (
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)

    position.roll = euler[0]/np.pi*180                  #roll
    position.pitch = -(euler[1]/np.pi*180)              #pitch
    position.dip = (-(euler[2]-(np.pi/2)))/np.pi*180    #yaw

    updateMarkerPose(odometry.pose.pose)

    publishPosition()

    return



def location_listener_node():
    rospy.init_node("location_listener_node", anonymous=True)

    global position_publisher, marker_publisher
    global drone_marker, position 
    
    drone_marker = Marker()
    initDroneMarker()
    position = GPSFix()

    position_publisher = rospy.Publisher('drone_position', GPSFix, queue_size=10)
    marker_publisher = rospy.Publisher('drone_marker', Marker, queue_size=10)

    rospy.Subscriber("mavros/global_position/local", Odometry, updateAttitude)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, updateLocation)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        location_listener_node()
    except rospy.ROSInterruptException:
        pass
