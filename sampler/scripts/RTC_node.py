#!/usr/bin/env python3

import rospy
import busio
import adafruit_pcf8523
import time
import board
import datetime
from watersampling_msgs.msg import RTC_time


class rtc_class():

    def __init__(self, rate):
        rospy.init_node('rtc_node', anonymous=True)
        
        self.myI2C = busio.I2C(board.SCL, board.SDA)
        self.rtc = adafruit_pcf8523.PCF8523(self.myI2C)
        
        self.rtc_publisher = rospy.Publisher('/watersampling/rtc_time', RTC_time, queue_size=1)
        self.rate = rate
        
        self.rtc_time()
    
    def rtc_time(self):
        r = rospy.Rate(self.rate)
        rtc_msg = RTC_time()

        while not rospy.is_shutdown():
            t = self.rtc.datetime
            #unix_time = datetime.datetime(t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec).timestamp()
            rtc_msg.header.stamp = rospy.Time.now()
            #rtc_msg.unix.data = int(unix_time)
            rtc_msg.year.data = t.tm_year
            rtc_msg.month.data = t.tm_mon
            rtc_msg.day.data = t.tm_mday
            rtc_msg.hours.data = t.tm_hour
            rtc_msg.min.data = t.tm_min
            rtc_msg.sec.data = t.tm_sec
                
            self.rtc_publisher.publish(rtc_msg)
                
            r.sleep()        
           
if __name__ == '__main__':
    rtc_class(1) #Publishes every second
