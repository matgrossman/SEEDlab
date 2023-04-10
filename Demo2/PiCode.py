import smbus
import time
import board
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import threading
import numpy as np
from math import sqrt

bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04
phi = 5

#sends both phi and distance, using the offset to determine what is being sent
def sendPhi(numToSend,offset):
    try:
        toSend = int(numToSend * 100)
        byteArray = toSend.to_bytes(offset,'big')
        print(toSend)
        print(list(byteArray))                                                                                                             
        bus.write_i2c_block_data(address, offset, list(byteArray))
    except:
        print("error")

 #calculation functions for phi and distance
def getPhi(corners):
    xAvg = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0])/4.0
    phi = (53.5/resolution[0])*xAvg
    return phi
def getEdgeLength(corners):
    edgeLen = cv2.arcLength(corners, True)/4
    dis = (588.5*1.968)/edgeLen
    return dis

#camera setup
resolution = (688,480)
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 24
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParam = cv2.aruco.DetectorParameters_create()
rawCapture = PiRGBArray(camera, size=resolution)
time.sleep(0.1)

#camera loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):        
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (corners,ids,rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParam)
    cv2.aruco.drawDetectedMarkers(gray, corners,ids)
    cv2.imshow('video',gray)
#   Dummy variable if no marker is found
    if ids == None:
        x = 0
#   Calculate and send distance and angle calculations. Offset timers to ensure I2C isn't busy on send
    else:
        x = 1
        phi = getPhi(corners[0])
        threading.Timer(0.05,sendPhi,args=(phi,4)).start()
        distance = getEdgeLength(corners[0])
        threading.Timer(0.15,sendPhi,args=(distance,2)).start()
    keyPress = cv2.waitKey(1) & 0xFF
#   Exit key
    if keyPress == ord('q'):
        cv2.destroyAllWindows()
        exit(0)
#   press to lock in white balancing
    elif keyPress == ord('c'):
        camera.shutter_speed = camera.exposure_speed
        g = camera.awb_gains
        camera.exposure_mode = 'off'
        camera.awb_mode = 'off'
        camera.awb_gains = g
    #Test print for distance calculation
    elif keyPress == ord('r'):
        print(getEdgeLength(corners[0]))
    rawCapture.truncate(0)