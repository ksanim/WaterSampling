#!/usr/bin/python3

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
from watersampling_msgs.msg import Setpoint
from geometry_msgs.msg import PoseStamped, Quaternion
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import ParamValue, Waypoint, PositionTarget
from mavros_test_common_Tweaked import MavrosTestCommonTweaked                                  # Imports the Class containing the subscriber and publisher functions
from pymavlink import mavutil                                                    # Imports mavlink Python utility functions
from six.moves import xrange                                                     # Imports xrange function which is basically range for python2
from std_msgs.msg import Header
from threading import Thread
from quat_lib import eul2rotm, quat2eul, quat2rotm, eul2quat
from read_from_plan_file import ReadPlanFile
from pd_controller_class import PD_Controller_Class
from gps_utils import GPS_utils

class MavrosOffboardPosctl(MavrosTestCommonTweaked):                                # Name_of_class(Base_Object) A class takes a base object as an argument. \
                                                                                    #Any function of the base object can be called with the super function.
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def __init__(self, *args):
        rospy.init_node('waypoint_sender_node', anonymous=True)
        super().__init__()                            # It's calling the __init__() function of the parent Class MavrosTestCommonTweaked
        self.DnH = PD_Controller_Class(MavrosTestCommonTweaked)
        
        self.mission_waypoints_LLA = dict()
        self.mission_waypoints_ENU = dict()
#        self.pos = PoseStamped()
        # Preparing messages for /mavros/setpoint_raw/local topic
        self.cmd_msg = PositionTarget()
        self.cmd_msg.coordinate_frame = PositionTarget().FRAME_BODY_NED
        self.cmd_msg.type_mask = PositionTarget().IGNORE_PX + \
                            PositionTarget().IGNORE_PY + \
                            PositionTarget().IGNORE_PZ + \
                            PositionTarget().IGNORE_AFX + \
                            PositionTarget().IGNORE_AFY + \
                            PositionTarget().IGNORE_AFZ + \
                            PositionTarget().IGNORE_YAW          # this is setting up the command publisher to ignore position or acceleration commands and only to take velocity commands and yaw rate.
        self.cmd_msg.velocity.x = 0.0
        self.cmd_msg.velocity.y = 0.0
        self.cmd_msg.velocity.z = 0.0
        self.cmd_msg.yaw_rate = 0.0
        
        self.setpoint_msg = Setpoint()
        
        self.radius = 0.2
        self.yaw = 0
        self.setpoint = [self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_from_global.pose.pose.position.z, self.yaw]
        self.reached = False
        self.mission_end = False
        self.sampling_flag_data = 0
        self.sampling_depth = 0.04
        self.inputs = [0,0,0,0]
        self.counter = 0
        
#        self.pos_setpoint_pub = rospy.Publisher(
#            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.vel_cmd_pub = rospy.Publisher(\
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.origin_pub = rospy.Publisher(\
            'mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher(\
            '/watersampling/setpoint', Setpoint, queue_size=1)
#        self.pos_setpoint_pub = rospy.Publisher(
#            'watersampling/setpoint_publisher', PoseStamped, queue_size=1)



        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)                                                 # This waits until all the topics are correctly being subscribed.
        self.log_topic_vars()

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_setpoint_raw)
        self.pos_thread.start()
        
        self.ctl_thread = Thread(target=self.posctl)
        self.ctl_thread.start()
        
        self.depth_check_thread = Thread(target=self.depth_check)
        self.depth_check_thread.start()
        
    #
#%%    # Helper methods
            
    def send_setpoint_raw(self):
        
        rate = rospy.Rate(30)  # Hz
        
        while not rospy.is_shutdown():
            
            if self.start:
          	    self.inputs = self.DnH.PD_controller(self.setpoint)
            self.cmd_msg.velocity.x = self.inputs[0]
            self.cmd_msg.velocity.y = self.inputs[1]
            self.cmd_msg.velocity.z = self.inputs[2]
            self.cmd_msg.yaw_rate = self.inputs[3]
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.vel_cmd_pub.publish(self.cmd_msg)
            
            self.setpoint_msg.header.stamp = rospy.Time.now()
            self.setpoint_msg.position.x = self.setpoint[0]
            self.setpoint_msg.position.y = self.setpoint[1]
            self.setpoint_msg.position.z = self.setpoint[2]
            self.setpoint_msg.yaw = np.rad2deg(self.setpoint[3])
            self.setpoint_pub.publish(self.setpoint_msg)
            
        
        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def distance_to_wp(self, lat, lon, alt):
        """alt(ellipsoid): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
#        alt_d = abs(alt - self.altitude.amsl)
        alt_d = abs(alt - self.global_position.altitude)
        
        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d, alt_d

    def is_at_position(self, x, y, z, offset):
        """offset: meters, (x,y,z) in ENU frame"""                                                    # rp.logdebug is just like rp.loginfo but used for debugging purpose.
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_from_global.pose.pose.position.z))

#        dxy, dz = self.distance_to_wp(lat,lon,alt)                              # Check using longitude, latitude, altitude using great circle distance.
#       Check using x,y,z using pithagoras
        desired = np.array((x, y))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y))

        dxy = np.linalg.norm(desired - pos)
        dz = abs(z - self.local_from_global.pose.pose.position.z)

        return dxy <= offset and dz < 0.3

    def reach_position(self, x, y, z, timeout):
        """
        x: lat
        y: lon
        z: ellipsoid alt
        timeout: seconds, maximum allowable time for achieving each intermediate setpoint 
        """
        
        target = self.u.geo2enu(x, y, z)
        current = self.u.geo2enu(self.global_position.latitude, self.global_position.longitude, self.global_position.altitude)
        dxy, dz = self.distance_to_wp(x,y,z)
        
    # Generate intermediate waypoints every 1.5 meters apart        
        n = int(dxy/1.5)
        if n < 2:
            n =  2
        xx = np.linspace(self.local_position.pose.position.x, target[0,0], n)
        yy = np.linspace(self.local_position.pose.position.y, target[1,0], n)

        rospy.loginfo("========================")
        rospy.loginfo(
            "attempting to reach position | latitude: {0}, longitude: {1}, altitude: {2} | current position latitude: {3}, longitude: {4}, altitude: {5}".
            format(x, y, z, self.global_position.latitude,
                   self.global_position.longitude,
                   self.global_position.altitude))

        # does it reach the position in 'timeout' seconds?

        for k in range(n):
            self.setpoint = [xx[k], yy[k], target[2,0], self.yaw]               # x, y, z, yaw
            
            loop_freq = 5  # Hz
            rate = rospy.Rate(loop_freq)

            for i in xrange(timeout * loop_freq):
                if self.is_at_position(xx[k], yy[k], target[2,0], self.radius):
                    rospy.loginfo("Intermediate position {0} of {1} reached | seconds: {2} of {3}".format(
                        k+1,n,i / loop_freq, timeout))
    
                    break
    
    
                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    rospy.logerr(e)
        
        if self.is_at_position(target[0,0], target[1,0], target[2,0], self.radius):
            self.reached = True
            rospy.loginfo("Waypoint Achieved, descending to water level")
        else:
            self.reached = False
            rospy.logerr("Failed to reach waypoint")
        
        self.counter = self.counter+1

    def sweep_position(self, x, y, timeout):
        """
        x: ENU x
        y: ENU y
        timeout: seconds, maximum allowable time for achieving each intermediate setpoint 
        """
        
        desired = np.array((x, y))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y))

        dxy = np.linalg.norm(desired - pos)     

    # Generate intermediate waypoints every 0.5 meters apart        
        n = int(dxy/0.5)
        if n < 2:
            n =  2
        xx = np.linspace(self.local_position.pose.position.x, desired[0], n)
        yy = np.linspace(self.local_position.pose.position.y, desired[1], n)

        rospy.loginfo("========================")
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1} | current position x: {2}, y: {3}".
            format(x, y, self.local_position.pose.position.x,
                   self.local_position.pose.position.y))

        # does it reach the position in 'timeout' seconds?

        for k in range(n):
            self.setpoint[0] = xx[k]
            self.setpoint[1] = yy[k]
            
            loop_freq = 5  # Hz
            rate = rospy.Rate(loop_freq)

            for i in xrange(timeout * loop_freq):
                if self.is_at_position(xx[k], yy[k], self.setpoint[2], self.radius) and self.inlet_depth >= self.sampling_depth:
                    rospy.loginfo("Intermediate position {0} of {1} reached | seconds: {2} of {3}".format(
                        k+1,n,i / loop_freq, timeout))
    
                    break
                elif self.inlet_depth < self.sampling_depth:
                    self.descend(10)
    
                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    rospy.logerr(e)


    def descend(self, timeout):
        if self.inlet_depth < self.sampling_depth and self.EKF.estimate_z.data > 0.5:
            self.setpoint[2] = self.setpoint[2] - 0.2
        
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            dz = abs(self.setpoint[2] - self.local_from_global.pose.pose.position.z)
            if dz < 0.1:
                break


            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
    #
    # Test method
    #
    def posctl(self):
        
#        rospy.loginfo("Please arm the drone to set the EKF origin")
        while(self.origin == None):
          #  if self.state.armed:
            self.center = GeoPointStamped()
            rospy.loginfo("Waiting for EKF Origin fix")
            rospy.sleep(10)
            self.center.header.frame_id = "geo"
            self.center.position.latitude = self.home_position.geo.latitude
            self.center.position.longitude = self.home_position.geo.longitude
            self.center.position.altitude = self.home_position.geo.altitude
            self.center.header.stamp = rospy.Time.now()
            self.origin_pub.publish(self.center)
            
            rospy.sleep(1)
        rospy.loginfo("Origin fixed!")
#%%        """Extract waypoints (both LLA and ENU) from a plan file and store into a dictionary."""
        """Test mission"""

        mission_dir = rospy.get_param('/waypoint_sender_node/mission_directory')
        mission_file = rospy.get_param('/waypoint_sender_node/file_name')
        
        if mission_file == 'None':
            rospy.logerr("usage: waypoint_sender.launch mission_file:=mission_file.plan")
            return
        mission_file = mission_dir + mission_file
        
        rospy.loginfo("reading mission {0}".format(mission_file))
        
        f = ReadPlanFile(mission_file)
        try:
            wps = f.read_mission()
        except IOError as e:
            rospy.logerr(e)
        
#%%        # Store the LLA waypoints in a dictionary with their sequence as their key.                 
        for index, waypoint in enumerate(wps):
        
            # only check position for waypoints where this makes sense
            if (waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT or
                    waypoint.frame == Waypoint.FRAME_GLOBAL):
                alt = waypoint.z_alt
                if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:#                                            # alt = rel_alt
                    alt += self.global_position.altitude - self.altitude.relative                                                      # set_alt_ellipsoid = rel_alt + home_alt_ellipsoid
                elif waypoint.frame == Waypoint.FRAME_GLOBAL:#                                                  # alt = set_alt_amsl
#                    alt += self.home_position.geo.altitude - (self.altitude.amsl - self.altitude.relative)      # set_alt_ellipsoid = set_alt_amsl + (home_alt_ellipsoid - home_alt_amsl) 
                    rospy.logerr("Altitude frame not supported")
                    return
            self.mission_waypoints_LLA[index] = (waypoint.x_lat, waypoint.y_long, alt)
        
           
        # Convert the mission_waypoints to ENU frame from LLA frame
        self.u = GPS_utils()
        origin = self.u.ecef2geo(self.origin.position.latitude, self.origin.position.longitude, self.origin.position.altitude)
        self.u.setENUorigin(origin[0], origin[1], origin[2])
        
        for index in range(len(self.mission_waypoints_LLA)):
            target = self.u.geo2enu(self.mission_waypoints_LLA[index][0], self.mission_waypoints_LLA[index][1], self.mission_waypoints_LLA[index][2])
            self.mission_waypoints_ENU[index] = (target[0,0], target[1,0], target[2,0])
            
            rospy.loginfo("Waypoint no. {0} | latitude: {1}, longitude: {2}, Altitude: {3:.3f} | x: {4:.3f}, y: {5:.3f}, z: {6:.3f}"\
                          .format(index, self.mission_waypoints_LLA[index][0], \
                                  self.mission_waypoints_LLA[index][1], \
                                  self.mission_waypoints_LLA[index][2], \
                                  self.mission_waypoints_ENU[index][0], \
                                  self.mission_waypoints_ENU[index][1], \
                                  self.mission_waypoints_ENU[index][2]))
        
        rospy.loginfo("Ready for mission!")
        
        while not rospy.is_shutdown():

            rate = rospy.Rate(1)  # Hz
            
            if self.start and not self.mission_end:
#                self.log_topic_vars()                                                # Logs all the important topics on the console.
                rospy.loginfo("Running mission")
                                                                                    # Measure the current yaw and fix the yaw to that
                rpy = quat2eul(self.local_position.pose.orientation.w,\
                self.local_position.pose.orientation.x,\
                self.local_position.pose.orientation.y,\
                self.local_position.pose.orientation.z,)
                self.yaw = rpy[2]
#                rospy.loginfo("yaw fixed to: {0} degrees".format(np.degrees(self.yaw)))
                  
     
                
                if self.sampling.flag.data == 0 and self.sampling.main_channel.data == False:
                    # GOTO waypoint 1
                    if self.counter == 0:
                        self.reach_position(self.mission_waypoints_LLA[0][0],\
                                            self.mission_waypoints_LLA[0][1],\
                                            self.mission_waypoints_LLA[0][2], 2)
                    self.descend(10)                                        # self.descend(timeout)
                    
                elif self.sampling.main_channel.data == True:
                    # SWEEP
                    for i in range(len(self.mission_waypoints_ENU)-1):
                        self.sweep_position(self.mission_waypoints_ENU[i+1][0],\
                                            self.mission_waypoints_ENU[i+1][1], 2)
                        if i == len(self.mission_waypoints_ENU)-1:
                            self.mission_end = True
                   
            else:
                if self.state.mode == "OFFBOARD":
                    rospy.loginfo("mission stopped")
                    self.set_mode("POSCTL", 5)
            
            rate.sleep()

    def depth_check(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(5)  # Hz
            if self.sampling.main_channel.data == True and self.EKF.estimate_z.data < 0.5:
                self.setpoint[2] = self.mission_waypoints_ENU[0][2]
                rospy.logerr("Oops! Dangerous Altitude, going up")
            if self.sampling.main_channel.data == True and self.mission_end:
                self.setpoint[2] = self.mission_waypoints_ENU[0][2]
                rospy.loginfo("Mission ended, going up")
                break
            rate.sleep()


if __name__ == '__main__':
    MavrosOffboardPosctl(MavrosTestCommonTweaked)
