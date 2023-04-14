#!/usr/bin/env python3
import serial

file = open('data_from_arduino.txt', 'w')
i=0

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

            file.write(line)
            file.write('\n')
            i=i+1
            #print(i)
            if (i>10):
                break
    file.close()
