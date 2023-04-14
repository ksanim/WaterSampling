from gpiozero import PWMLED
from time import sleep

led = PWMLED(20)

led.value = 1.0
sleep(60)
led.value = .5
sleep(60)

#for a in range(10):
#    for b in range(101):
#        led.value = b / 100.0
#        sleep(0.01)
#    for b in range(101):
#        print(b)
#        led.value = (100-b) / 100.0
#        sleep(0.02)
