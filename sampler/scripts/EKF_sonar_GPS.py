#!/usr/bin/env python3
import rospy as rp
import threading
import numpy as np
import math
from scipy.stats.distributions import chi2

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from watersampling_msgs.msg import EKFInfo
from watersampling_msgs.msg import SonarStamped


class ExtendedKalmanFilter():
    """
        Extended Kalman Filter

        n  -> Number of States
        X  -> State
        F  -> State Transition Matrix Discrete
        H  -> Measurement Matrix Discrete
        P  -> Error Covariance Matrix
        Q  -> Process Noise Matrix
        R  -> Measurement Noise Matrix
            
    """
    def __init__(self, rate):
        rp.init_node("EKF_node")
        self.rate = rate
        # Initialize system constants
        self.r = 0.10            # = 100 mm, distance between sonar and pixhawk in f550 testbed.
        self.start_time = 0          # The time step for the sonar sensor reading, the lowest frequency among all sensors
        self.dt = 0
        self.ax_b = 0
        self.ay_b = 0
        self.az_b = 0 
              
        # Initialize R and Q matrices
        self.Q = np.matrix(np.diag([0.01, 0.0488, 0.0004, 0.000324, 0.00027]))       # [z; z_dot; roll(phi); pitch(theta); offset]
        self.R = np.matrix(np.diag([0.00001, 0.005, 0.00005, 0.0002]))               # [GPS, Sonar, Roll, Pitch]
        self.F = np.matrix(np.zeros((5, 5)))
        self.H = np.matrix(np.zeros((4, 5)))
        self.K = None
        self.quat = list(np.zeros(4))                                               # [qw, qx, qy, qz]
        
        self.sonar_readings = []
        
        #%% 1. Extended Kalman Filter
        # Initialize the variables to store the estimated state,
        # the error covariance matrix, the process and the measurement noise
        # covariance
        
        
        self.X_est_EKF = np.matrix(np.zeros((5,1))) # [z; z_dot; roll(phi); pitch(theta); offset]
        self.P_EKF = np.matrix(np.diag([0.01, 0.01, 0.01, 0.01, 0.1]))
        self.y = np.matrix(np.zeros((4,1)))             #[GPS; Sonar; roll; pitch]
        self.y_exp = np.matrix(np.zeros((4,1)))         #[GPS; Sonar; roll; pitch]
        self.u = np.matrix(np.zeros((3,1)))             #[acc_z; roll_rate; pitch_rate]
        
        # ROS Subscribers
        self.imu_sub = rp.Subscriber(
            '/mavros/imu/data', Imu, self.imuCallback, queue_size=1)
        self.position_sub = rp.Subscriber(
            '/mavros/global_position/local', Odometry, self.positionCallback, queue_size=1)
        self.sonar_sensor_sub = rp.Subscriber(
            '/watersampling/sonar_dist', SonarStamped, self.sonarCallback, queue_size=1)
        
        # ROS Publisher
        self.EKF_info_pub = rp.Publisher(
            '/watersampling/EKF_info', EKFInfo, queue_size=1)           #rp.Publisher('topic name', message_type, queue_size)
        
        rp.loginfo("EKF started")
        
        t = threading.Thread(target=self.EKFInfoPublisher)
        t.start()
        
        rp.spin()
        
    def positionCallback(self, msg):
        self.y[0,0] = msg.pose.pose.position.z
        self.quat[0] = msg.pose.pose.orientation.w
        self.quat[1] = msg.pose.pose.orientation.x
        self.quat[2] = msg.pose.pose.orientation.y
        self.quat[3] = msg.pose.pose.orientation.z
        
        #print('x: ',self.quat[1],'y: ', self.quat[2],'z: ', self.quat[3],'w: ', self.quat[0])
    def imuCallback(self, msg):
        self.ax_b = msg.linear_acceleration.x        # acceleration_x in body frame
        self.ay_b = msg.linear_acceleration.y        # acceleration_y in body frame
        self.az_b = msg.linear_acceleration.z        # acceleration_z in body frame
        
        self.u[1,0] = msg.angular_velocity.x           # roll_rate
        self.u[2,0] = msg.angular_velocity.y           # pitch_rate
        
        #print('x: ',self.u[1,0],'y: ', self.u[2,0])
    def sonarCallback(self, msg):
        #self.y[1,0] = msg.distance.data                    # take raw sonar data as measurement
        self.y[1,0] = self.max_filter(msg.distance.data, 5) # take max filtered sonar data as measurement
        
    def max_filter(self, dist, window):
       # print(self.sonar_readings)
        self.sonar_readings.append(dist)
        while len(self.sonar_readings) > window:
            self.sonar_readings.pop(0)
        
        return max(self.sonar_readings)
        
    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y
    
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)
    
        return X, Y, Z
    
    def fetchF_EKF(self):
        # X = [x1; x2; x3; x4; x5]
        # X = [z; z_dot; roll; pitch; offset]
        
        self.F[0, 0] = 1 #df1dx1
        self.F[0, 1] = self.dt #df1dx2
        self.F[0, 2] = 0 #df1dx3
        self.F[0, 3] = 0 #df1dx4
        self.F[0, 4] = 0 #df1dx5
        
        self.F[1, 0] = 0 #df2dx1
        self.F[1, 1] = 1 #df2dx2
        self.F[1, 2] = (np.cos(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0])*self.ay_b - \
        np.sin(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0])*self.az_b)*self.dt #df2dx3
        
        self.F[1, 3] = (-np.cos(self.X_est_EKF[3,0])*self.ax_b - \
        np.sin(self.X_est_EKF[2,0])*np.sin(self.X_est_EKF[3,0])*self.ay_b - \
        np.cos(self.X_est_EKF[2,0])*np.sin(self.X_est_EKF[3,0])*self.az_b)*self.dt #df2dx4
        
        self.F[1, 4] = 0 #df2dx5
        
        self.F[2, 0] = 0 #df3dx1
        self.F[2, 1] = 0 #df3dx2
        self.F[2, 2] = 1 #df3dx3
        self.F[2, 3] = 0 #df3dx4
        self.F[2, 4] = 0 #df3dx5
        
        self.F[3, 0] = 0 #df4dx1
        self.F[3, 1] = 0 #df4dx2
        self.F[3, 2] = 0 #df4dx3
        self.F[3, 3] = 1 #df4dx4
        self.F[3, 4] = 0 #df4dx5
        
        self.F[4, 0] = 0 #df5dx1
        self.F[4, 1] = 0 #df5dx2
        self.F[4, 2] = 0 #df5dx3
        self.F[4, 3] = 0 #df5dx4
        self.F[4, 4] = 1 #df5dx5

    def fetchH_EKF(self):
        # X = [x1; x2; x3; x4; x5]
        # X = [z; z_dot; roll(phi); pitch(theta); offset; offset_dot]
        # h1 = GPS+FCU measurement
        # h2 = Sonar measurement
        # h3 = roll angle measurement
        # h4 = pitch angle measurement
        
        # self.r = 0.26 # 260 mm, distance between 2 landing legs is 520 mm.
        # self.dt = 0.067
        cpct = np.cos(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0])
    
        self.H[0, 0] = 1 # dH1dx1
        self.H[0, 1] = 0 # dH1dx2
        self.H[0, 2] = 0 #dH1dx3
        self.H[0, 3] = 0 #dH1dx4
        self.H[0, 4] = 1 #dH1dx5
        
        self.H[1, 0] = 1/cpct # dH2dx1
        self.H[1, 1] = 0 # dH2dx2
        self.H[1, 2] = self.r + (self.X_est_EKF[0,0] + self.r*np.sin(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0]))*np.tan(self.X_est_EKF[2,0])/cpct #dH2dx3
        self.H[1, 3] = self.X_est_EKF[0,0]*np.tan(self.X_est_EKF[3,0])/cpct #dH2dx4
        self.H[1, 4] = 0 #dH2dx5
        
        self.H[2, 0] = 0 # dH3dx1
        self.H[2, 1] = 0 # dH3dx2
        self.H[2, 2] = 1 #dH3dx3
        self.H[2, 3] = 0 #dH3dx4
        self.H[2, 4] = 0 #dH3dx5
        
        self.H[3, 0] = 0 # dH4dx1
        self.H[3, 1] = 0 # dH4dx2
        self.H[3, 2] = 0 #dH4dx3
        self.H[3, 3] = 1 #dH4dx4
        self.H[3, 4] = 0 #dH4dx5

    def stateEstimate(self):
        if self.start_time == 0:
            self.start_time = rp.get_time()
            self.dt = 0.066
        else:
            self.dt = rp.get_time()-self.start_time
        
        # Update the acceleration inputs and transform them to the world frame
        self.u[0,0] = (-np.sin(self.X_est_EKF[3,0]))*self.ax_b + \
        np.sin(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0])*self.ay_b + \
        np.cos(self.X_est_EKF[2,0])*np.cos(self.X_est_EKF[3,0])*self.az_b - 9.8;        # acceleration_z in the world frame
        
        # Update the estimated states        
        self.X_est_EKF[0, 0] = self.X_est_EKF[0, 0] + self.X_est_EKF[1, 0]*self.dt
        self.X_est_EKF[1, 0] = self.X_est_EKF[1, 0] + self.u[0,0]*self.dt
        self.X_est_EKF[2, 0] = self.X_est_EKF[2, 0] + self.u[1,0]*self.dt
        self.X_est_EKF[3, 0] = self.X_est_EKF[3, 0] + self.u[2,0]*self.dt
        self.X_est_EKF[4, 0] = self.X_est_EKF[4, 0]
        
        # Update the error covariance matrix
        self.fetchF_EKF()
        self.P_EKF = self.F*self.P_EKF*self.F.T + self.Q
        
        angles = self.quaternion_to_euler_angle(self.quat[0], self.quat[1], self.quat[2], self.quat[3])      # returns euler angles in XYZ format in radians
       
        # Get Measurements
    #   self.y[0, 0] = GPS Altitude
    #   self.y[1, 0] = sonar_sensor
        self.y[2, 0] = angles[0]    # ROLL ANGLE IN RADIAN
        self.y[3, 0] = angles[1]    # PITCH ANGLE IN RADIAN
        
        # Get expected measurement
        self.y_exp[0, 0] = self.X_est_EKF[0, 0] + self.X_est_EKF[4, 0]
        self.y_exp[1, 0] = (self.X_est_EKF[0, 0]+self.r*np.sin(self.X_est_EKF[2, 0])*np.cos(self.X_est_EKF[3, 0]))/(np.cos(self.X_est_EKF[2, 0])*np.cos(self.X_est_EKF[3, 0]))
        self.y_exp[2, 0] = self.X_est_EKF[2, 0]
        self.y_exp[3, 0] = self.X_est_EKF[3, 0]
        
        # Compute H Matrix
        self.fetchH_EKF()
        
        # ---------------------------------------------------------------------
        # Vanilla EKF
        # ---------------------------------------------------------------------
    #     # Compute Kalman gain
    #     K = P_EKF*H'/(H*P_EKF*H' + R)
    #     
    #     # Correct state estimate
    #     self.X_est_EKF(:, i) = self.X_est_EKF(:, i) + K*(y(:, i) - y_exp(:, i))
    #     
    #     # Update the error covariance matrix
    #     P_EKF = (eye(5) - K*H)*P_EKF
        
        # ---------------------------------------------------------------------
        # Sequential Kalman Filter
        # ---------------------------------------------------------------------
            
        for i in range(self.y.shape[0]):
            # chi square test (Measurement gating)
            Sigma_z = (self.H[i,:]*self.P_EKF*self.H[i,:].T+self.R[i,i])
            Innovation = self.y[i,0] - self.y_exp[i, 0]
            
            if Innovation.T*(np.linalg.inv(Sigma_z))*Innovation > chi2.ppf(0.995, df=self.y.shape[0]):
                continue
            
            # Kalman Gain Update
            self.K = self.P_EKF*self.H[i,:].T*(np.linalg.inv(Sigma_z))
            
            # Correct State estimate
            self.X_est_EKF[:, 0] = self.X_est_EKF[:, 0] + self.K*Innovation
            
            # Update Error covariance
            self.P_EKF = (np.matrix(np.eye(5)) - self.K*self.H[i,:])*self.P_EKF        
        # ---------------------------------------------------------------------
        
        # update the start_time
        self.start_time = rp.get_time()
        
        
    def EKFInfoPublisher(self,):
        r = rp.Rate(self.rate)
        EKF_msg = EKFInfo()

        while not rp.is_shutdown():
            self.stateEstimate()

            # Populate and publish pump message
            EKF_msg.header.stamp = rp.Time.now()
            EKF_msg.estimate_z.data = self.X_est_EKF[0, 0]
            EKF_msg.estimate_z_dot.data = self.X_est_EKF[1, 0]
            EKF_msg.estimate_roll.data = self.X_est_EKF[2, 0]
            EKF_msg.estimate_pitch.data = self.X_est_EKF[3, 0]
            EKF_msg.estimate_bias.data = self.X_est_EKF[4, 0]
            EKF_msg.time.data = self.dt

            self.EKF_info_pub.publish(EKF_msg)
            
            r.sleep()

if __name__ == '__main__':
    ExtendedKalmanFilter(15)

#%% All the functions are down below

#
##%% Secondmax Filter
#def secondmax_filter(data, window):
#    data_buffer = []
#    filtered_data = zeros(size(data))
#    for i = 1 : length(data):
#        data_buffer = [data_buffer; data(i)]
#        if length(data_buffer) > window:
#            data_buffer = data_buffer(2:)
#        
#        if length(data_buffer) > 3:
#            mx = max(data_buffer(1),data_buffer(2))
#            secondmx = min(data_buffer(1),data_buffer(2))
#    
#            for j = 3 : length(data_buffer):
#                if data_buffer(j)>mx:
#                    secondmx=mx
#                    mx=data_buffer(j)
#                elif data_buffer(j)>secondmx and mx ~= data_buffer(j):
#                    secondmx=data_buffer(j)
#        else
#            mx = max(data_buffer)
#            secondmx = mx
#        
#            filtered_data(i) = secondmx
#            


