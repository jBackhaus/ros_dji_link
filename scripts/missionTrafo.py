#!/usr/bin/env python
#Created by Jan Backhaus on 12.12.18 as part of the project ros_dji_link.

import rospy
import std_msgs.msg
from std_msgs.msg import String
from threading import Thread
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import Waypoint
from mission_control.msg import Mission
from dji_link.srv import *
import time


def MR_Speed_Waypoint(speed):
    #set Speed used for Path to this new Waypoint
    #mavLink sets this as separated Message of type "Waypoint" but with command "178" - "DO_CHANGE_SPEED"
    mrWaypoint = Waypoint()
    mrWaypoint.command = 178 #DO_CHANGE_SPEED
    mrWaypoint.frame = 0    #GLOBAL
    mrWaypoint.param1 = 1   #Ground Speed, DJI drones are capable of this
    mrWaypoint.param2 = speed
    mrWaypoint.param3 = -1  #unchanged (no) throttle-Value
    mrWaypoint.param4 = 0   #absolute value
    return(mrWaypoint)

def MR_Photo_Distance_Waypoint(distance):
    #set photo-distance (not defined in MissionControl-Node) 
    #mavLink sets this as separated Message of type "Waypoint" but with command "206" - "DO_SET_CAM_TRIGG_DIST"
    mrWaypoint = Waypoint()
    mrWaypoint.command = 206 #DO_SET_CAM_TRIGG_DIST
    mrWaypoint.frame = 0    #GLOBAL
    mrWaypoint.param1 = distance   #Distance, 0 to stop taking Photos
    mrWaypoint.param2 = 0   #"1" => triger camera once
    return(mrWaypoint)

def MR_Nav_Waypoint(mcWaypoint):
    #NAV-Waypoint
    mrWaypoint = Waypoint()
    mrWaypoint.command = 16 #NAV_WAYPOINT
    mrWaypoint.frame = 0   #GLOBAL
    mrWaypoint.autocontinue = True
    mrWaypoint.param1 = mcWaypoint.time_to_wait
    mrWaypoint.x_lat = mcWaypoint.point.lat
    mrWaypoint.y_long = mcWaypoint.point.lng
    mrWaypoint.z_alt = mcWaypoint.rel_height
    return(mrWaypoint)

def MR_Home_Waypoint(mcWaypoint):
    #Home-Waypoint
    mrWaypoint = Waypoint()
    mrWaypoint.command = 179 #DO_SET_HOME
    mrWaypoint.frame = 0   #GLOBAL
    mrWaypoint.autocontinue = True
    if (mcWaypoint.point.lat == 0):   #no Home is set in MissionControl
        mrWaypoint.param1 = 1   #use current location
    else:
        mrWaypoint.param1 = 0
        mrWaypoint.x_lat = mcWaypoint.point.lat
        mrWaypoint.y_long = mcWaypoint.point.lng
        mrWaypoint.z_alt = mcWaypoint.rel_height

    return(mrWaypoint)

def MR_Gimbal_Pitch_Waypoint(pitch):
    #set camera gimbal pitch to param
    #mavLink sets this as separated Message of type "Waypoint" but with command "205" - "DO_MOUNT_CONTROL"
    mrWaypoint = Waypoint()
    mrWaypoint.command = 205 #DO_MOUNT_CONTROL
    mrWaypoint.frame = 0    #GLOBAL
    mrWaypoint.param1 = pitch   #pitch
    mrWaypoint.param2 = 0   #roll
    mrWaypoint.param3 = 0   #yaw
    mrWaypoint.param4 = 0   #altitude (not used in this mode)
    mrWaypoint.x_lat = 0    #latitude (not used in this mode)
    mrWaypoint.y_long = 0   #longitude (not used in this mode)
    mrWaypoint.z_alt = 2    #MAV_MOUNT_MODE 2 => MAVLINK_TARGETING => "Start Mavlink Roll,Pitch,Yaw control with stabilization"
    return(mrWaypoint)



def MC_to_MR(req):
    mrMission = WaypointList()
    mrMission.current_seq = 0
    mcMission = req.mcMission.waypoints

    #set Home if given in MissionControl
    if (req.mcMission.home.point.lat != 0):   #Home Position is set
        mrMission.waypoints.append(MR_Home_Waypoint(req.mcMission.home))
    
    #set Gimbal Pitch from Param 'gimbalPitch'
    mrMission.waypoints.append(MR_Gimbal_Pitch_Waypoint(rospy.get_param('gimbalPitch')))

    counter = 0
    speed = 0
    cam_trigger_dist = 0


    for mcWaypoint in mcMission:

        #set Speed, if it has changed
        if (mcWaypoint.speed != speed): #change Speed
            speed = mcWaypoint.speed
            mrMission.waypoints.append(MR_Speed_Waypoint(speed))
        
        #set Nav_Waypoint
        mrMission.waypoints.append(MR_Nav_Waypoint(mcWaypoint))
        counter += 1

        #if cam_trigger_distance would be defined in MissionControl, it could be implemented here. Same way as speed.
        #until implementation, camera_trigger_distance is set after the first Waypoint
        #value is set in param 'imageDistance'
        if (counter == 1):
            mrMission.waypoints.append(MR_Photo_Distance_Waypoint(rospy.get_param('imageDistance')))
        
    #quit Photos
    mrMission.waypoints.append(MR_Photo_Distance_Waypoint(0)) 
    return mcTOmrResponse(mrMission)




def mission_trafo():
    rospy.init_node("missionTrafo", anonymous=True)
    mctomr = rospy.Service('getMRMission', mcTOmr, MC_to_MR)
    rospy.spin()




if __name__ == '__main__':
    try:
        mission_trafo()
    except rospy.ROSInterruptException:
        pass
