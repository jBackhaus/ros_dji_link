#!/usr/bin/env python

#Created by Jan Backhaus on 12.12.18 as part of the project ros_dji_link.

import sys
import rospy
from dji_link.srv import *
from mission_control.srv import GetMission
from mavros_msgs.srv import WaypointPush, CommandBool

def upload_mission(req):
    if (req.request == 1):
        #loadMission (from MissionControl-Node)
        try:
            rospy.wait_for_service('GetMission',2)
        except Exception, e:
            print "Timeout: Service 'GetMission' not available: %s"%e
            return uploadMissionResponse(False, 0)

        try: 
            getMission = rospy.ServiceProxy('GetMission', GetMission)
            retval = getMission(True)
            mcMission = retval.mission
        except rospy.ServiceException, e:
            print "Error calling getMission: %s"%e
            return uploadMissionResponse(False, 0)


        #call Service to transform mcMission to mrMission (from MissionTrafo-Node)
        try:
            rospy.wait_for_service('getMRMission',2)
        except Exception, e:
            print "Timeout: Service 'getMRMission' not available: %s"%e
            return uploadMissionResponse(False, 0)
        try:
            getMRMission = rospy.ServiceProxy('getMRMission', mcTOmr)
            retval = getMRMission(mcMission)
            mrMission = retval.mrMission
        except rospy.ServiceException, e:
            print "Error calling getMRMission: %s"%e
            return uploadMissionResponse(False, 0)



        #call Service to push mission to drone (from MAVROS-Node)
        try:
            rospy.wait_for_service('mavros/mission/push',2)
        except Exception, e:
            print "Timeout: Service 'mavros/mission/push' not available: %s"%e
            return uploadMissionResponse(False, 0)
        try:
            pushMission = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            retval = pushMission(mrMission.current_seq, mrMission.waypoints)
            success = retval.success
            wp_transferred = retval.wp_transfered
        except rospy.ServiceException, e:
            print "Error calling mavros/mission/push: %s"%e
            return uploadMissionResponse(False, 0)

        return uploadMissionResponse(success, wp_transferred)

def start_mission(req):
    if (req.start == 1):
        #Call Service to arm drone (from MAVROS-Node)
        try:
            rospy.wait_for_service('mavros/cmd/arming',2)
        except Exception, e:
            print "Timeout: Service 'mavros/smd/arming' not available: %s"%e
            return startMissionResponse(False)
        try:
            armDrone = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            retval = armDrone(True)
            success = retval.success
        except rospy.ServiceException, e:
            print "Error calling mavros/mission/push: %s"%e
            return startMissionResponse(False)
        return startMissionResponse(True)


def dji_link_node():
    rospy.init_node("dji_link", anonymous=True)
    mctomr = rospy.Service('uploadMission', uploadMission, upload_mission)
    startCommand = rospy.Service('startMission', startMission, start_mission)

    rospy.spin()




if __name__ == '__main__':
    try:
        dji_link_node()
    except rospy.ROSInterruptException:
        pass