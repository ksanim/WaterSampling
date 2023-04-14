#!/usr/bin/python3
import threading

import rospy as rp
import numpy as np
from quat_lib import eul2rotm, quat2eul, quat2rotm

from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger, SetBool

#from sensor_msgs.msg import Joy
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RCNode():
  
  _RC_TRIGGER_PD = 10           # 2005 - 1494 - 982
  _RC_HIGH_PD = 1494
  _RC_TRIGGER_OFFBOARD = 12      # 982 - 1494 - 2005
  _RC_HIGH_OFFBOARD = 1494
    
  def __init__(self, rate):
    rp.init_node('rc_node')

    self.rate = rate

    # Check that services are available
    rp.loginfo('Checking that services are available')
    rp.wait_for_service('/mavros/set_mode')
    rp.wait_for_service('/mavros/cmd/arming')
    #rp.wait_for_service('/enable_controller')
    rp.loginfo('MavROS and NMPC services are available')

    # Setpoints
    self.setpoint = [0,0,1.0,0]             # [x, y, z, yaw]

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
    
    
#    # Velocity cmds
#    self.vel_x_cmd = 0.0
#    self.vel_y_cmd = 0.0
#    self.vel_z_cmd = 0.0
#    self.vel_q_cmd = 0.0

    # Control switches
#    self.arm_bttn = 0
#    self.ofb_bttn = 0
#    self.pd_bttn = 0

    # Status
    self.connected = False
    self.armed = False
    self.mode = None
    self.pd_enabled = False

    # Service clients
#    self.set_mode_serv = rp.ServiceProxy('/mavros/set_mode', SetMode)
#    self.arm_serv = rp.ServiceProxy('/mavros/cmd/arming', CommandBool)
    #self.ctrl_serv = rp.ServiceProxy('/enable_controller', SetBool)

    # Subscribers
    self.state_sub = rp.Subscriber('/mavros/state', State, self.stateCallback, queue_size=1)
    self.rc_sub = rp.Subscriber('/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)
#    self.joy_sub = rp.Subscriber('/joy', Joy, self.joyCallback, queue_size=1)
#    self.position_sub = rp.Subscriber('/mavros/global_position/local', Odometry, self.positionCallback, queue_size=1)
    self.position_sub = rp.Subscriber('/mavros/local_position/pose', PoseStamped, self.positionCallback, queue_size=1)
    
    # Publishers
    self.vel_cmd_pub = rp.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    t = threading.Thread(target=self.commandPublisher)
    t.start()

    rp.spin()

  def rcCallback(self, msg):
    if msg.channels[self._RC_TRIGGER_PD] == self._RC_HIGH_PD:
      self.pd_enabled = True
    else:
      self.pd_enabled = False
            
  def stateCallback(self, msg):
    self.connected = msg.connected
    self.armed = msg.armed
    self.mode = msg.mode

#  def joyCallback(self, msg):
#    self.vel_x_cmd =  msg.axes[3]                   # Right stick up-down
#    self.vel_y_cmd =  msg.axes[2]                   # Right stick right-left
#    self.vel_z_cmd =  msg.axes[1]                   # Left stick up-down
#    self.vel_q_cmd =  msg.axes[0]                   # Left stick left-right
#
#    self.arm_bttn = msg.buttons[5]                  # RB button
#    self.ofb_bttn = msg.buttons[4]                  # LB button
#    self.pd_bttn = msg.buttons[7]                   # RT button

#-----------------------uncomment this when flying inside optitrack-----------    
  def positionCallback(self, msg):                   # position callback from /mavros/local_position/pose
    self.pos_world[0,0] = msg.pose.position.x        # position x in world frame
    self.pos_world[1,0] = msg.pose.position.y        # position y in world frame
    self.pos_world[2,0] = msg.pose.position.z        # position z in world frame
    self.quat[0] = msg.pose.orientation.w
    self.quat[1] = msg.pose.orientation.x
    self.quat[2] = msg.pose.orientation.y
    self.quat[3] = msg.pose.orientation.z
#-----------------------uncomment this when flying with GPS-------------------    
#  def positionCallback(self, msg):                         # position callback from /mavros/global_position/local
#    self.pos_world[0,0] = msg.pose.pose.position.x        # position x in world frame
#    self.pos_world[1,0] = msg.pose.pose.position.y        # position y in world frame
#    self.pos_world[2,0] = msg.pose.pose.position.z        # position z in world frame
#    self.quat[0] = msg.pose.pose.orientation.w
#    self.quat[1] = msg.pose.pose.orientation.x
#    self.quat[2] = msg.pose.pose.orientation.y
#    self.quat[3] = msg.pose.pose.orientation.z
    
  def clip_input(self, value):
    if value >= 1.0:
      value = 1.0
    elif value <= -1.0:
      value = -1.0
    else:
      value = value
      
    return value

  def PD_controller(self):
    
    Kp_z = 1.8024           # Proportional gain for z input
    Kd_z = 0.0324           # Derivative gain for z input
    Kp_xy = 0.4282          # Proportional gain for xy input
    Kd_xy = 0.0077          # Derivative gain for xy input
    
    rpy = quat2eul(self.quat[0], self.quat[1], self.quat[2], self.quat[3])
    yaw = rpy[2]

    R_W_B = eul2rotm(rpy[2], rpy[1], rpy[0])               # Rotation matrix for transforming body to world
    R_B_W = R_W_B.T                                        # Rotation matrix for transforming world to body
    
    
    self.error_x = self.setpoint[0] - self.pos_world[0,0]
    self.error_y = self.setpoint[1] - self.pos_world[1,0]
    self.error_z = self.setpoint[2] - self.pos_world[2,0]
    self.error_q = self.setpoint[3] - yaw

    x_input_world = Kp_xy*self.error_x + Kd_xy*(self.error_x-self.prev_error_x)
    y_input_world = Kp_xy*self.error_y + Kd_xy*(self.error_y-self.prev_error_y)
    z_input_world = Kp_z*self.error_z + Kd_z*(self.error_z-self.prev_error_z)
    q_input_world = Kp_xy*self.error_q + Kd_xy*(self.error_q-self.prev_error_q)
    
    input_world = np.matrix([x_input_world, y_input_world, z_input_world]).T
    input_body = R_B_W*input_world                                              # rotated the inputs in world frame to body frame
    
    self.vel_x_input = self.clip_input(input_body[0,0])
    self.vel_y_input = self.clip_input(input_body[1,0])
    self.vel_z_input = self.clip_input(input_body[2,0])
    self.vel_q_input = self.clip_input(q_input_world)
    
    self.prev_error_x = self.error_x
    self.prev_error_y = self.error_y 
    self.prev_error_z = self.error_z
    self.prev_error_q = self.error_q
    

  def commandPublisher(self,):
    r = rp.Rate(self.rate)
    cmd_msg = PositionTarget()
    cmd_msg.coordinate_frame = PositionTarget().FRAME_BODY_NED
    cmd_msg.type_mask = PositionTarget().IGNORE_PX + \
                        PositionTarget().IGNORE_PY + \
                        PositionTarget().IGNORE_PZ + \
                        PositionTarget().IGNORE_AFX + \
                        PositionTarget().IGNORE_AFY + \
                        PositionTarget().IGNORE_AFZ + \
                        PositionTarget().IGNORE_YAW          # this is setting up the command publisher to ignore position or acceleration commands and only to take velocity commands and yaw rate.
    cmd_msg.velocity.x = 0.0
    cmd_msg.velocity.y = 0.0
    cmd_msg.velocity.z = 0.0
    cmd_msg.yaw_rate = 0.0

    while not rp.is_shutdown():
      if self.connected:
        # Make sure we are in the correct flight mode
#        if self.ofb_bttn == 0.0 and self.mode != 'POSCTL':
#          self.set_mode_serv(0, 'POSCTL')
#        elif self.ofb_bttn == 1.0 and self.mode != 'OFFBOARD':
#          self.set_mode_serv(0, 'OFFBOARD')

#        # Arm if requested and not armed
#        if self.arm_bttn == 1.0 and not self.armed:
#          self.arm_serv(True)
        
#        if self.pd_bttn == 1.0 and not self.pd_enabled:
#         # self.ctrl_serv(True)
#          self.pd_enabled = True
#        elif self.pd_bttn == 0.0 and self.pd_enabled:
#         # self.ctrl_serv(False)
#          self.pd_enabled = False

        if self.armed and not self.pd_enabled:
          cmd_msg.velocity.x = 0.0
          cmd_msg.velocity.y = 0.0
          cmd_msg.velocity.z = 0.0
          cmd_msg.yaw_rate = 0.0

          cmd_msg.header.stamp = rp.Time.now()
          self.vel_cmd_pub.publish(cmd_msg)
          
        elif self.armed and self.pd_enabled:
          self.PD_controller()
          cmd_msg.velocity.x = self.vel_x_input
          cmd_msg.velocity.y = self.vel_y_input
          cmd_msg.velocity.z = self.vel_z_input
          cmd_msg.yaw_rate = self.vel_q_input

          cmd_msg.header.stamp = rp.Time.now()
          self.vel_cmd_pub.publish(cmd_msg)

      else:
        rp.loginfo('Vehicle not connected')

      r.sleep()


if __name__ == '__main__':
    JoyRCNode(30)