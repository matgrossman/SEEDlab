import smbus
import time
import board
#import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import threading
import numpy as np
import struct
from math import sqrt


# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
currentMarker = 1
# This is the address we setup in the Arduino Program
address = 0x04
phi = 5
#def lcdDisplay():
#    print("test")
#    lcd.clear()
#    lcd.message = "Marker detected:\n" + str(phi)
#    threading.Timer(0.2,lcdDisplay).start()
lineBusy = False;
def sendPhi(numToSend,offset):
    global lineBusy
    if not lineBusy:
        lineBusy = True;
        try:
            toSend = int(numToSend * 100)
            byteArray = toSend.to_bytes(offset,'big')
            print(toSend)
            print(list(byteArray))                                                                                                             
            bus.write_i2c_block_data(address, offset, list(byteArray))
        except:
            print("send error")
        lineBusy = False;
        
def readAtMarker():
    global lineBusy
    global currentMarker
    if not lineBusy:
        lineBusy = True;
        try:
            currentMarker = bus.read_byte_data(address, 1)
        except:
            print("read error")
        lineBusy = False;
    threading.Timer(1,readAtMarker).start()
 
def getPhi(corners):
    xAvg = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0])/4.0
    phi = (53.5/resolution[0])*xAvg
    return phi
def getEdgeLength(corners):
    edgeLen = cv2.arcLength(corners, True)/4
    dis = (588.5*1.968)/edgeLen
    return dis
resolution = (688,480)
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 24
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParam = cv2.aruco.DetectorParameters_create()
rawCapture = PiRGBArray(camera, size=resolution)
threading.Timer(1,readAtMarker).start()
time.sleep(0.1)

#threading.Timer(0.2,lcdDisplay).start()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):        
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (corners,ids,rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParam)
    cv2.aruco.drawDetectedMarkers(gray, corners,ids)
    cv2.imshow('video',gray)
    if ids == None:
        x = 0
    else:
        print(ids)
        for i in range(len(ids)):
            if ids[i][0] == currentMarker:
                print("dawg thats the right one")
                phi = getPhi(corners[i])
                threading.Timer(0.01,sendPhi,args=(phi,4)).start()
                distance = getEdgeLength(corners[i])
                threading.Timer(0.05,sendPhi,args=(distance,2)).start()
            else:
                print("Psych that's the wrong number")
    keyPress = cv2.waitKey(1) & 0xFF
    if keyPress == ord('q'):
        cv2.destroyAllWindows()
        exit(0)
    elif keyPress == ord('c'):
        print("lol")
        camera.shutter_speed = camera.exposure_speed
        g = camera.awb_gains
        camera.exposure_mode = 'off'
        camera.awb_mode = 'off'
        camera.awb_gains = g
    elif keyPress == ord('r'):
#        print('works')
        print(getEdgeLength(corners[0]))
    elif keyPress == ord('v'):
        camera.exposure_mode = 'on'
        camera.awb_mode = 'on'
    rawCapture.truncate(0)