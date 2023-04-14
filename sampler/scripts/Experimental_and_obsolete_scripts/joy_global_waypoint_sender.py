#!/usr/bin/env python2

#
# @base author Andreas Antener <andreas@uaventure.com>
# @author Kazi Ragib Ishraq Sanim <ksanim@github.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division
                                                                                 # This module includes all the latest functions which were not present in the previous version in Python.\
                                                                                 #Its releases are in Python version 2.1. The basic idea of the __future__ module is to help migrate to use Python 3.X features. 
PKG = 'px4'

import rospy
import math
import numpy as np
import sys
import os
from geometry_msgs.msg import PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import ParamValue, Waypoint
from mavros_test_common_Tweaked import MavrosTestCommonTweaked                                  # Imports the Class containing the subscriber and publisher functions
from pymavlink import mavutil                                                    # Imports mavlink Python utility functions
from six.moves import xrange                                                     # Imports xrange function which is basically range for python2
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion      # Function to convert a quaternion to Euler angle
from read_from_plan_file import ReadPlanFile
from gps_utils import GPS_utils

class MavrosOffboardPosctlTest(MavrosTestCommonTweaked):                                # Name_of_class(Base_Object) A class takes a base object as an argument. \
                                                                                    #Any function of the base object can be called with the super function.
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def __init__(self, *args):
        rospy.init_node('waypoint_sender_node', anonymous=True)
        super(MavrosOffboardPosctlTest, self).__init__()                            # It's calling the __init__() function of the parent Class MavrosTestCommonTweaked
        
        self.mission_waypoints = dict()
        self.pos = GeoPoseStamped()
        self.radius = 1
        
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

                # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)                                                 # This waits until all the topics are correctly being subscribed.
        self.log_topic_vars()

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
#        self.pos_thread.daemon = True                                            # A daemon is a long-running background process that answers requests for services
        self.pos_thread.start()
        
        self.ctl_thread = Thread(target=self.posctl)
        self.ctl_thread.start()
        

        
#        rospy.spin()
    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
#            print("send_pos_called")
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass                                                            # pass = Do Nothing

    def distance_to_wp(self, lat, lon, alt):
        """alt(amsl): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(alt - self.altitude.amsl)
        
        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d, alt_d

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""                                                    # rp.logdebug is just like rp.loginfo but used for debugging purpose.
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        dxy, dz = self.distance_to_wp(x,y,z)

        return dxy <= offset and dz < 0.3

    def reach_position(self, index, timeout):
        """timeout(int): seconds"""
#        self.mission_waypoints[index] = (waypoint.x_lat, waypoint.y_long, alt)
        
        # For demo purposes we will lock yaw/heading to East.
#        yaw_degrees = 0  # East
#        yaw = math.radians(yaw_degrees)
#        quaternion = quaternion_from_euler(0, 0, yaw)
#        self.pos.pose.orientation = Quaternion(*quaternion)
#        
#        # Set the yaw/heading towards the next waypoint.
#
        u = GPS_utils()
        u.setENUorigin(self.home_position.geo.latitude, self.home_position.geo.longitude, self.home_position.geo.altitude)
        target = u.geo2enu(self.mission_waypoints[index][0], self.mission_waypoints[index][1], self.mission_waypoints[index][2])
        current = u.geo2enu(self.global_position.latitude, self.global_position.longitude, self.global_position.altitude)
        numerator = current[1]-target[1]
        denominator = current[0]-target[0]
        yaw = np.pi + math.atan2(numerator, denominator)                        # Adding pi because the drone thinks that the West is 0 yaw
        print("target yaw = ", np.degrees(yaw))
        quaternion = quaternion_from_euler(0, 0, yaw)
        
        
#        current = euler_from_quaternion([self.local_position.pose.orientation.w,\
#                                 self.local_position.pose.orientation.x,\
#                                 self.local_position.pose.orientation.y,\
#                                 self.local_position.pose.orientation.z])
        
#        if abs(current[0] - yaw) >= 0.0174:
#            self.pos.pose.position.latitude = self.global_position.latitude
#            self.pos.pose.position.longitude = self.global_position.longitude
#            self.pos.pose.position.altitude = self.mission_waypoints[index][2]
#            self.pos.pose.orientation = Quaternion(*quaternion)
#            print("if")
        
        self.pos.pose.position.latitude = self.mission_waypoints[index][0]
        self.pos.pose.position.longitude = self.mission_waypoints[index][1]
        self.pos.pose.position.altitude = self.mission_waypoints[index][2]
        self.pos.pose.orientation = Quaternion(*quaternion)
        print("else")
        
        rospy.loginfo(
            "attempting to reach position | latitude: {0}, longitude: {1}, AMSL altitude: {2} | current position latitude: {3}, longitude: {4}, AMSL altitude: {5}".
            format(self.mission_waypoints[index][0],
                   self.mission_waypoints[index][1],
                   self.mission_waypoints[index][2],
                   self.global_position.latitude,
                   self.global_position.longitude,
                   self.altitude.amsl))

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.latitude,
                                   self.pos.pose.position.longitude,
                                   self.pos.pose.position.altitude, self.radius):
#            if self.is_at_position(self.mission_waypoints[index][0],
#                                   self.mission_waypoints[index][1],
#                                   self.mission_waypoints[index][2], self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                #rospy.sleep(5)
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    #
    # Test method
    #
    def posctl(self):
        """Test offboard position control"""
        """Test mission"""
        if len(sys.argv) < 2:
            rospy.logerr("usage: joy_global_waypoint_sender.py mission.plan")
            return

#        self.mission_name = sys.argv[1]
#        path_to_mission = rp.get_param('/waypoint_sender_node/mission_directory')
        
        path = os.path.dirname(os.path.realpath(__file__))
        path_split = path.split('/')
        path_to_mission = ''
        for i in range(len(path_split) - 2):
            path_to_mission += path_split[i] + '/'
        path_to_mission += 'missions/'
        
        mission_file = path_to_mission + sys.argv[1]

        rospy.loginfo("reading mission {0}".format(mission_file))
        
        f = ReadPlanFile(mission_file)
        try:
            wps = f.read_mission()
        except IOError as e:
            rospy.logerr(e)
                   
        for index, waypoint in enumerate(wps):
        # only check position for waypoints where this makes sense
            if (waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT or
                    waypoint.frame == Waypoint.FRAME_GLOBAL):
                alt = waypoint.z_alt
                if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                    alt += self.altitude.amsl - self.altitude.relative
            self.mission_waypoints[index] = (waypoint.x_lat, waypoint.y_long, alt)
            
            rospy.loginfo("Waypoint no. {0} | latitude: {1}, longitude: {2}, Altitude: {3}"\
                          .format(index, self.mission_waypoints[index][0], \
                                  self.mission_waypoints[index][1], \
                                  self.mission_waypoints[index][2]))

            
        while not rospy.is_shutdown():

            rate = rospy.Rate(10)  # Hz
            
            if self.start == True:
                self.log_topic_vars()                                                # Logs all the important topics on the console.
                rospy.loginfo("run mission")
#            # exempting failsafe from lost RC to allow offboard
#                rcl_except = ParamValue(1<<2, 0.0)
#                self.set_param("COM_RCL_EXCEPT", rcl_except, 5)                         # Specify modes in which RC loss is ignored and the failsafe action not triggered. 0: Mission, 1: Hold, 2: Offboard
                self.set_mode("OFFBOARD", 5)


                for i in xrange(len(self.mission_waypoints)):
                    self.reach_position(i, 20)

    
            else:
                if self.state.mode == "OFFBOARD":
                    self.set_mode("POSCTL", 5)
                
            rate.sleep()
#        self.set_mode("AUTO.LAND", 5)
#        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
#                                   45, 0)
#        self.set_arm(False, 5)


if __name__ == '__main__':
    MavrosOffboardPosctlTest(MavrosTestCommonTweaked)
