#!/usr/bin/env python3

import RPi.GPIO as gpio
import threading
import time
import rospy
from watersampling_msgs.msg import SonarStamped


class sonar():

    _TRIG = 12                                   #BCM pin 12, BOARD pin 32
    def __init__(self, rate):
        rospy.init_node('sonar', anonymous=True)
        
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)                     # gives if a gpio pin is used as both input and output

        self.distance_publisher = rospy.Publisher('/watersampling/sonar_dist', SonarStamped, queue_size=1)
        self.rate = rate
        
        time.sleep(0.5)
        rospy.loginfo("------------------------sonar start")
        
        t = threading.Thread(target=self.dist_sendor)
        t.start()
        
        rospy.spin()
    
    def measure_dist(self):
        gpio.setup(self._TRIG, gpio.OUT)
        gpio.output(self._TRIG, 0)
        time.sleep(0.02)                    # The sonar sleeps for 20 miliseconds
        #time.sleep(1)
        gpio.output(self._TRIG, 1)
        time.sleep(0.000005)                # The sonar sends out a ping for 5 microseconds
        gpio.output(self._TRIG, 0)
        gpio.setup(self._TRIG, gpio.IN)
        
        pulse_end = None
        while gpio.input(self._TRIG) == 0 :       # Records the time immediately after the Ping is sent. 
                                            #After the ping is sent, the trig pin stays LOW for an instant.
                                            #then it stays high until it receives the echo.
            pulse_start = time.time()       # time.time() returns time in seconds since the epoch as a floating point number
        while gpio.input(self._TRIG) == 1 :       # until the echo returns, it will record the time of flight.
            pulse_end = time.time()
        
        if pulse_end == None:               # if the sensor is disconnected physically, pulse_end will remain None type.
            return 0
        
        pulse_duration = pulse_end - pulse_start
        
        distance_raw = pulse_duration * 170    # 2s = vt, therefore, s = (v/2)*t. Speed of sound is v=340 m/s or 34000 cm/s. Therefore, v/2 = 17000 cm/s
        if pulse_duration >=0.01746:
            rospy.logwarn("sonar time out")
            return 0
        elif distance_raw > 3 or distance_raw<=0:
            rospy.logwarn("sonar out of range")
            return 0
        self.distance = round(distance_raw, 3)
        return 1
        print ('Distance : %f cm'%distance)                   # Takes a reading 15 times per second. That means duration for 1 data point is 66.7 ms.
        
 
        
    def dist_sendor(self,):
        r = rospy.Rate(self.rate)
        sonar_msg = SonarStamped()

        while not rospy.is_shutdown():
            if self.measure_dist():
        
                sonar_msg.header.stamp = rospy.Time.now()
                sonar_msg.distance.data = self.distance
                
                self.distance_publisher.publish(sonar_msg)
            else:
                rospy.logwarn("Sensor read failed. Will try again")
                pass
                
            r.sleep()
        
        gpio.cleanup()
        
           
if __name__ == '__main__':
    sonar(15)
