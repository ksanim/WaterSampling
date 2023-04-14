#!/usr/bin/env python2
from __future__ import division
                                                                                 # This module includes all the latest functions which were not present in the previous version in Python.\
                                                                                 # Its releases are in Python version 2.1. The basic idea of the __future__ module is to help migrate to use Python 3.X features. 
#import unittest                                                                  # Imports the Python unit testing framework. setUp and tearDown comes fromt this framework. \
                                                                                 # The number of individual tests are defined with methods whose names start with the letters test.\
                                                                                 # This naming convention informs the test runner about which methods represent tests.
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, \
                            WaypointList, RCIn
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush
from watersampling_msgs.msg import EKFInfo, MS5837Stamped, PumpInfo
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu, Joy
from six.moves import xrange

class MavrosTestCommonTweaked():

# Auto Channel for F550
#    _RC_TRIGGER_MISSION = 4           # 2006 - 1494 - 982
#    _RC_HIGH_MISSION = 1494

# Auto Channel for Aurelia
    _RC_TRIGGER_MISSION = 10           # 2005 - 1494 - 982
    _RC_HIGH_MISSION = 1494
    _RC_OFFBOARD = 14
    _RC_HIGH_OFFBOARD = 2005            # 982 - 2005
    
    
    def __init__(self, *args):
        
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.local_from_global = Odometry()
        self.mav_type = None
        
        # Control switches

        self.arm_bttn = 0
        self.ofb_bttn = 0
        self.pd_bttn = 0
        self.start = False
        self.origin = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'local_pos',
                'mission_wp', 'state', 'imu', 'global_pos', 'home_pos', 'local_from_global'
            ]
        }
        #'global_pos', 'home_pos',                                              # Uncomment when flying with GPS

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.local_from_global_sub = rospy.Subscriber('mavros/global_position/local', Odometry,self.local_from_global_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints',\
                                               WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        self.gp_origin_sub = rospy.Subscriber('/mavros/global_position/gp_origin',\
                                              GeoPointStamped, self.gp_origin_callback, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy',\
                                        Joy, self.joyCallback, queue_size=1)
        self.sampler_sub = rospy.Subscriber('/watersampling/sampler_pump_info',\
                                            PumpInfo, self.samplerCallback, queue_size=1)
        self.EKF_sub = rospy.Subscriber('/watersampling/EKF_info',\
                                        EKFInfo, self.EKFCallback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/watersampling/depth_sensor',\
                                          MS5837Stamped, self.depthCallback, queue_size = 1)
        self.rc_sub = rospy.Subscriber('/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)

        self.rc_sub = rospy.Subscriber('/mavros/rc/in',\
                                       RCIn, self.rcCallback, queue_size=1)

        

    #
    # Callback functions
    #
    def rcCallback(self, msg):
        if msg.channels[self._RC_TRIGGER_MISSION] == self._RC_HIGH_MISSION:
            self.start = True
        else:
            self.start = False
        
        if msg.channels[self._RC_OFFBOARD] != self._RC_HIGH_OFFBOARD and self.state.mode == "OFFBOARD":
            self.set_mode("POSCTL", 5)                                      # Switches to POSCTL when the offboard switch is off.
    
    def joyCallback(self, msg):
        self.arm_bttn = msg.buttons[5]                  # RB button
        self.ofb_bttn = msg.buttons[6]                  # LT button
        self.pd_bttn = msg.buttons[7]                   # RT button
        
        if self.pd_bttn == 1.0 and not self.start:
          self.start = True
          #self.set_mode("OFFBOARD", 5)
        elif self.pd_bttn == 0.0 and self.start:
          self.start = False
         # self.set_mode("POSCTL", 5)
        if self.arm_bttn == 1.0 and not self.state.armed:
            self.set_arm(True, 5)
        
        if self.ofb_bttn == 0.0 and self.state.mode != 'POSCTL':
          self.set_mode("POSCTL", 5)
        elif self.ofb_bttn == 1.0 and self.state.mode != 'OFFBOARD':
          self.set_mode("OFFBOARD", 5)
          
    def gp_origin_callback(self, msg):
        self.origin = msg
            
    def samplerCallback(self, msg):
        self.sampling = msg
    
    def EKFCallback(self, msg):
        self.EKF = msg
        
    def depthCallback(self, msg):
        self.depth_sensor = msg
        self.inlet_depth = msg.depth.data
    
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True                                 # sub_topics_ready['alt'] will become True once the altitude callback is called.

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True                           # sub_topics_ready['ext_state'] will become True once the altitude callback is called.

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True                          # sub_topics_ready['global_pos'] will become True once the altitude callback is called.

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True                                 # sub_topics_ready['imu'] will become True once the altitude callback is called.

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True                            # sub_topics_ready['home_pos'] will become True once the altitude callback is called.

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True                           # sub_topics_ready['local_pos'] will become True once the altitude callback is called.

    def local_from_global_callback(self, data):
        self.local_from_global = data
        if not self.sub_topics_ready['local_from_global']:
            self.sub_topics_ready['local_from_global'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True                          # sub_topics_ready['mission_wp'] will become True once the altitude callback is called.

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True                               # sub_topics_ready['state'] will become True once the altitude callback is called.

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def set_param(self, param_id, param_value, timeout):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
#        if not simulation_ready:
#            rospy.signal_shutdown("topics are not ready")

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_from_global:\n{}".format(self.local_from_global))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")
