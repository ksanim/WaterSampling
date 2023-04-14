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
from geometry_msgs.msg import PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import ParamValue
from mavros_test_common_Tweaked import MavrosTestCommonTweaked                                  # Imports the Class containing the subscriber and publisher functions
from pymavlink import mavutil                                                    # Imports mavlink Python utility functions
from six.moves import xrange                                                     # Imports xrange function which is basically range for python2
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler                             # Function to convert a quaternion to Euler angle


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
        
        self.pos = PoseStamped()
        self.radius = 1
        
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

                # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)                                                 # This waits until all the topics are correctly being subscribed.
        

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
#        self.pos_thread.daemon = True                                            # A daemon is a long-running background process that answers requests for services
        self.pos_thread.start()
        
        self.ctl_thread = Thread(target=self.posctl)
        self.ctl_thread.start()
        
        self.log_topic_vars()
        
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

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""                                                    # rp.logdebug is just like rp.loginfo but used for debugging purpose.
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))

        return np.linalg.norm(desired - pos) < offset                           # norm = sqrt(x^2 + y^2 + z^2) which is basically the 3D distance between the desired and the current position

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
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
        
        while not rospy.is_shutdown():
#            print("posctl_called")
            rate = rospy.Rate(5)  # Hz
            if self.start == True:
                self.log_topic_vars()                                                # Logs all the important topics on the console.
                rospy.loginfo("run mission")
#            # exempting failsafe from lost RC to allow offboard
#                rcl_except = ParamValue(1<<2, 0.0)
#                self.set_param("COM_RCL_EXCEPT", rcl_except, 5)                         # Specify modes in which RC loss is ignored and the failsafe action not triggered. 0: Mission, 1: Hold, 2: Offboard
                self.set_mode("OFFBOARD", 5)
            
                positions = ((0, 0, 2), (5, 5, 2), (5, -5, 2), (-5, -5, 2),
                             (0, 0, 0))
    
        
                for i in xrange(len(positions)):
                        self.reach_position(positions[i][0], positions[i][1],
                                        positions[i][2], 30)
    
            else:
                if self.state.mode == "OFFBOARD":
                    self.set_mode("POSCTL", 5)
                
            rate.sleep()


if __name__ == '__main__':
    MavrosOffboardPosctlTest(MavrosTestCommonTweaked)
