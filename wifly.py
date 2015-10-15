#!/usr/bin/python


import serial
import time
import json

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600)

def parseDict(dict):
    print "Sensor 1 reading: " + str(dict["sensor_1"])
    

y = ""
while 1:
    if ser.inWaiting() != 0:
        y += ser.read(ser.inWaiting())
        print y
        if '}' in y:
            y = '[' + y + ']'
            try:
                z = json.loads(y)
                parseDict(z[0])
                print str(z)
                print type(z)
                ser.write(y)
                print "JSON sent"
                y = ""
            except:
                ser.write(y)
                y = ""
                print "reg sent"
    time.sleep(0.01)

