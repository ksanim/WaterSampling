#!/usr/bin/python3
import rospy as rp
import threading

from statistics import mean, median

from pump_driver import Pump
from bottle_state import Bottle

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from mavros_msgs.msg import RCIn
from watersampling_msgs.msg import MS5837Stamped, PumpInfo


class SamplerNode():
    """
    A ROS node that monitors the water sensors and triggers the sampling
    """
   #check the /mavros/rc/in topic to find out which switch controls which channel
    _RC_TRIGGER_CHANNEL_MAIN = 10           # 2005 - 1494 - 982
    _RC_HIGH_MAIN = 1494
    _RC_TRIGGER_CHANNEL_SamplerBC = 12      # 982 - 1494 - 2005
    _RC_HIGH_SamplerC = 1494
    _RC_HIGH_SamplerB = 2005
    _RC_TRIGGER_CHANNEL_SamplerA = 6        # 1851 - 1494 -1136
    _RC_HIGH_SamplerA = 1494

    _MASTER_PUMP_PIN = 16   #BCM pin 16, BOARD pin 36
    _SAMPLING_PUMP_A = 26   #BCM pin 26, BOARD pin 37
    _SAMPLING_PUMP_B = 20   #BCM pin 20, BOARD pin 38
    _SAMPLING_PUMP_C = 19   #BCM pin 19, BOARD pin 35
    
    _MAIN_CHANNEL = 8       #BCM pin 8
    _BOTTLE_A = 7           #BCM pin 7
    _BOTTLE_B = 25          #BCM pin 25
    _BOTTLE_C = 1           #BCM pin 1

    def __init__(self, rate):
        rp.init_node("water_sampler_node")
        self.rate = rate

        # Pump instances
        self.master_pump = Pump(self._MASTER_PUMP_PIN)
        self.sampling_pump_a = Pump(self._SAMPLING_PUMP_A)
        self.sampling_pump_b = Pump(self._SAMPLING_PUMP_B)
        self.sampling_pump_c = Pump(self._SAMPLING_PUMP_C)
        
        # Bottle instances
        self.main_channel = Bottle(self._MAIN_CHANNEL)
        self.sampler_A = Bottle(self._BOTTLE_A)
        self.sampler_B = Bottle(self._BOTTLE_B)
        self.sampler_C = Bottle(self._BOTTLE_C)
        
        self.sampling_flag = 0
        self.depth_flag = 0
        self.counter = 0
        
        self.enable_sampler_main = False
        self.enable_sampler_C = False
        self.enable_sampler_B = False
        self.enable_sampler_A = False

# PARAMETERS        
        self.inlet_depth = 0.0
        self.sampling_depth = 0.2
        self.fluor_median = 0
        self.fluor_trigger = 100.0                                              # Need to change this value when going out in the real field.
        self.fluor_mode_record = 1
        self.fluor_mode_stop = 0

        # Fluorescence sensor
        self.fluorescence_readings = []
        self.fluorescence_list_size = 10

        # ROS Subscribers
        self.rc_sub = rp.Subscriber(
            '/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)
        self.depth_sensor_sub = rp.Subscriber(
            '/watersampling/depth_sensor', MS5837Stamped, self.depthSensorCallback, queue_size=1)
        self.fluor_sensor_sub = rp.Subscriber(
            '/sensor/fluor', Float32, self.fluorSensorCallback, queue_size=1)

        # ROS Publisher
        self.pump_info_pub = rp.Publisher(
            '/watersampling/sampler_pump_info', PumpInfo, queue_size=1)
        self.mode_pub = rp.Publisher(
            '/sensor/mode', Int16, queue_size=1)

        t = threading.Thread(target=self.pumpInfoPublisher)
        t.start()

        rp.spin()
    
    def mode_sendor(self, mode_number):
        mode = Int16()
        mode.data=mode_number
        self.mode_pub.publish(mode)
    
    def rcCallback(self, msg):
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_SamplerA] == self._RC_HIGH_SamplerA:
            self.enable_sampler_A = True
        else:
            self.enable_sampler_A = False
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_SamplerBC] == self._RC_HIGH_SamplerB:
            self.enable_sampler_B = True
        else:
            self.enable_sampler_B = False
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_SamplerBC] == self._RC_HIGH_SamplerC:
            self.enable_sampler_C = True
        else:
            self.enable_sampler_C = False
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_MAIN] == self._RC_HIGH_MAIN:
            self.enable_sampler_main = True
        else:
            self.enable_sampler_main = False

    def depthSensorCallback(self, msg):
        self.inlet_depth = msg.depth.data

    def fluorSensorCallback(self, msg):
        self.fluorescence_readings.append(msg.data)
        while len(self.fluorescence_readings) > self.fluorescence_list_size:
            self.fluorescence_readings.pop(0)

    def stateMachine(self,):
        if self.enable_sampler_main == True:
        
# Flushing routine-------------------------------------------------------------
            if self.counter == 0:
                rp.loginfo("Starting the flushing routine")
                self.master_pump.start()
                self.sampling_pump_a.start()
                self.sampling_pump_b.start()
                self.sampling_pump_c.start()
                
                rp.sleep(10)            # Flushing duration = 10 seconds
                
                self.master_pump.stop()
                self.sampling_pump_a.stop()
                self.sampling_pump_b.stop()
                self.sampling_pump_c.stop()
                
                self.counter = self.counter+1
                rp.loginfo("The lines have been flushed")
# Flushing routine ends--------------------------------------------------------

            if self.inlet_depth >= self.sampling_depth or self.enable_sampler_C == True:   #self.enable_sampler_C is the RC switch
                self.master_pump.start()
                
                #%% Check if there is water running in the circuit------------------------------
                
                if self.main_channel.is_full == True:
                    self.mode_sendor(self.fluor_mode_record)        # publishes mode = 1 to the /sensor/mode topic to start getting fluorescence measurements         
                
                #%% Calculate median of fluorescence window-------------------------------------
                if(len(self.fluorescence_readings) > 0):
                    self.fluor_median = median(self.fluorescence_readings)
                else:
                    self.fluor_median = 0                # Change this to zero when we the fluoresensor is connected to the Pi
                
                if(self.sampling_flag == 0 and self.fluor_median > self.fluor_trigger):
                    self.sampling_flag = 1

                self.depth_flag = 0
            else:
                self.depth_flag = 1
                self.master_pump.stop()

        else:
            self.sampling_flag = 0
            self.main_channel.is_full = False
            self.sampler_A.is_full = False
            self.sampler_B.is_full = False
            self.sampler_C.is_full = False
       
            self.master_pump.stop()
            self.mode_sendor(self.fluor_mode_stop)                    # publishes mode = 0 to the /sensor/mode topic to stop getting fluorescence measurements 

    def pumpInfoPublisher(self,):
        r = rp.Rate(self.rate)
        pump_msg = PumpInfo()

        while not rp.is_shutdown():
            self.stateMachine()

            # Populate and publish pump message
            pump_msg.header.stamp = rp.Time.now()
            pump_msg.flag.data = self.sampling_flag
            pump_msg.master_pump.data = self.master_pump.isrunning
            pump_msg.sampling_pump_a.data = self.sampling_pump_a.isrunning
            pump_msg.sampling_pump_b.data = self.sampling_pump_b.isrunning
            pump_msg.sampling_pump_c.data = self.sampling_pump_c.isrunning
            pump_msg.main_channel.data = self.main_channel.is_full
            pump_msg.sampler_A.data = self.sampler_A.is_full
            pump_msg.sampler_B.data = self.sampler_B.is_full
            pump_msg.sampler_C.data = self.sampler_C.is_full

            self.pump_info_pub.publish(pump_msg)
            
            r.sleep()


if __name__ == '__main__':
    SamplerNode(30)
