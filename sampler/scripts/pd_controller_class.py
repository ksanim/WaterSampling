#!/usr/bin/python3

import rospy as rp
import numpy as np
from quat_lib import eul2rotm, quat2eul, quat2rotm
from mavros_test_common_Tweaked import MavrosTestCommonTweaked 


class PD_Controller_Class(MavrosTestCommonTweaked):
  def __init__(self, *args):
    super().__init__() 
#    # Setpoints
#    self.setpoint = [0,0,1.0,0]             # [x, y, z, yaw]

    # Input cmds
    self.vel_x_input = 0.0
    self.vel_y_input = 0.0
    self.vel_z_input = 0.0
    self.vel_q_input = 0.0
    
    # Position and Orientation in world frame
    self.pos_world = np.matrix(np.zeros(3)).T
    self.pos_body = np.matrix(np.zeros(3)).T
    self.quat = np.zeros(4)

    # Error value initialization
    self.error_x = 0.0
    self.error_y = 0.0 
    self.error_z = 0.0
    self.error_q = 0.0
    
    self.prev_error_x = 0.0
    self.prev_error_y = 0.0 
    self.prev_error_z = 0.0
    self.prev_error_q = 0.0
    
    self.inlet_depth = 0.0
    self.vel_horz_max = 5.0
 
  def clip_input(self, value):
    if value >= self.vel_horz_max:
      value = self.vel_horz_max
    elif value <= -self.vel_horz_max:
      value = -self.vel_horz_max
    else:
      value = value
      
    return value

  def PD_controller(self, setpoint):
    
    self.pos_world[0,0] = self.local_position.pose.position.x        # position x in world frame
    self.pos_world[1,0] = self.local_position.pose.position.y        # position y in world frame
    
#    if self.inlet_depth >= self.sampling_depth:
#        self.pos_world[2,0] = self.EKF.estimate_z.data
#        rp.loginfo("Using EKF Altitude")
#    else:
    self.pos_world[2,0] = self.local_from_global.pose.pose.position.z        # position z in world frame
#        rp.loginfo("Using GPS Altitude")
    self.quat[0] = self.local_position.pose.orientation.w
    self.quat[1] = self.local_position.pose.orientation.x
    self.quat[2] = self.local_position.pose.orientation.y
    self.quat[3] = self.local_position.pose.orientation.z
      
    Kp_z = 1.8024           # Proportional gain for z input
    Kd_z = 0.0324           # Derivative gain for z input
    Kp_xy = 0.4282          # Proportional gain for xy input
    Kd_xy = 0.0077          # Derivative gain for xy input
    
    rpy = quat2eul(self.quat[0], self.quat[1], self.quat[2], self.quat[3])
    yaw = rpy[2]

    R_W_B = eul2rotm(rpy[2], rpy[1], rpy[0])               # Rotation matrix for transforming body to world
    R_B_W = R_W_B.T                                        # Rotation matrix for transforming world to body
    
    
    self.error_x = setpoint[0] - self.pos_world[0,0]
    self.error_y = setpoint[1] - self.pos_world[1,0]
    self.error_z = setpoint[2] - self.pos_world[2,0]
    self.error_q = setpoint[3] - yaw
    
    # Check for yaw_error greater than 180 degs or pi radians. If so, calculate the shortest error.
    if abs(self.error_q) > np.pi:
        if self.error_q > 0:
            self.error_q = self.error_q - 2*np.pi
        elif self.error_q < 0:
            self.error_q = self.error_q + 2*np.pi

    x_input_world = Kp_xy*self.error_x + Kd_xy*(self.error_x-self.prev_error_x)
    y_input_world = Kp_xy*self.error_y + Kd_xy*(self.error_y-self.prev_error_y)
    z_input_world = Kp_z*self.error_z + Kd_z*(self.error_z-self.prev_error_z)
    q_input_world = Kp_xy*self.error_q + Kd_xy*(self.error_q-self.prev_error_q)
    
    input_world = np.matrix([x_input_world, y_input_world, z_input_world]).T
    input_body = R_B_W*input_world                                              # rotated the inputs in world frame to body frame
    
    self.vel_x_input = self.clip_input(input_body[0,0])
    self.vel_y_input = self.clip_input(input_body[1,0])
    self.vel_z_input = self.clip_input(input_body[2,0])
    self.vel_q_input = q_input_world
    
    self.prev_error_x = self.error_x
    self.prev_error_y = self.error_y 
    self.prev_error_z = self.error_z
    self.prev_error_q = self.error_q
    
    return self.vel_x_input, self.vel_y_input, self.vel_z_input, self.vel_q_input




