#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 15:12:58 2022

@author: sanim
"""

from __future__ import division
import rospy
import json
from mavros_msgs.msg import Waypoint
from mavros_test_common_Tweaked import MavrosTestCommonTweaked 

class ReadPlanFile:
    def __init__(self, mission_filename):
        self.mission_filename = mission_filename
        rospy.loginfo("Reading from the plan file")
        
        
    def read_mission(self):
        wps = []
        with open(self.mission_filename, 'r') as f:
            for waypoint in self.read_plan_file(f):
                wps.append(waypoint)
                rospy.logdebug(waypoint)
    
        return wps


    def read_plan_file(self, f):
        d = json.load(f)
        if 'mission' in d:
            d = d['mission']
                                                                                    # yield is like return, but it can enable the function to continue execution from the point of it's last run
        if 'items' in d:
            for wp in d['items']:
                yield Waypoint(
                    is_current=False,
                    frame=int(wp['frame']),
                    command=int(wp['command']),
                    param1=float('nan'
                                 if wp['params'][0] is None else wp['params'][0]),
                    param2=float('nan'
                                 if wp['params'][1] is None else wp['params'][1]),
                    param3=float('nan'
                                 if wp['params'][2] is None else wp['params'][2]),
                    param4=float('nan'
                                 if wp['params'][3] is None else wp['params'][3]),
                    x_lat=float(wp['params'][4]),
                    y_long=float(wp['params'][5]),
                    z_alt=float(wp['params'][6]),
                    autocontinue=bool(wp['autoContinue']))
        else:
            raise IOError("no mission items")