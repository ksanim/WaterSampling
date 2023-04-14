import RPi.GPIO as GPIO
from time import sleep

from sampler.scripts.pump_driver import Pump

trigger_pin = 40
mastersampler = Pump(35)              # broadcom pin 19 (35), 16 (36), 26 (37), 20 (38) are being used, the pins in the brackets are physical pins 
sampler_A = Pump(36)
sampler_B = Pump(37)
sampler_C = Pump(38)
GPIO.setup(trigger_pin,GPIO.IN)

    
try:  
    while True:            # this will carry on until you hit CTRL+C  
        if GPIO.input(40): # if port 25 == 1  
            mastersampler.start()         # set port/pin value to 1/HIGH/True 
            print("motor is running") 
        else:   
            mastersampler.stop()         # set port/pin value to 0/LOW/False
            print("motor has stopped")  
        sleep(0.1)         # wait 0.1 seconds  
  
finally:                   # this block will run no matter how the try block exits  
    GPIO.cleanup()         # clean up after yourself  
