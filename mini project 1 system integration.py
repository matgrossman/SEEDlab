# mini project 1
# system integration
# recieve byte from camera function and send it to arduino over i2c
# Luke Stodghill

# SMBus and LCD Setup
import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# LCD setup
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()  # uses board.SCL and board.SDA
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


# Computer Vision Setup
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2


#Default setup for camera with resolution and arucoDict and Parameters for aruco detection
resolution = (640,480)
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParam = cv2.aruco.DetectorParameters_create()

def writeNumber(value):
    bus.write_byte(address, value)
    #bus.write_byte_data(address, 1, value)
    return -1

def readBlock():
    number = bus.read_i2c_block_data(address, 7, 2) # 2 is number of bytes in data, 7 is the sent offset
    # number = bus.read_byte_data(address, 0)
    return number

def getQuadrant(corners):
    xAvg = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0])/4.0
    phi = (53.5/resolution[0])*xAvg -26.75
    yPos = (corners[0][0][1]+corners[0][1][1] + corners [0][2][1] + corners [0][3][1])/4.0 - resolution[1]/2.0
    
    if yPos < 0.0:

        if phi < 0.0:
            return 0
        else:
            return 1

    elif yPos > 0.0:

        if phi < 0.0:
            return 2
        else:
            return 3

def calibrateCamera(camera):
    isochoice= 0
    while True:
        rawCapture = PiRGBArray(camera,size=resolution)
        
        camera.iso = 200
        time.sleep(2)
        camera.shutter_speed = camera.exposure_speed
        g = camera.awb_gains
        camera.exposure_mode = 'off'
        camera.awb_mode = 'off'
        camera.awb_gains = g
        lst = []
        lst.append(camera.capture_sequence(['image%02d.jpg' % i for i in range(5)]))
        print("displaying test images")
        break

#calibrateCamera(camera)
#
#rawCapture = PiRGBArray(camera, size=resolution)
#time.sleep(0.1)
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#    image = frame.array
#    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#    (corners,ids,rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParam)
#    cv2.aruco.drawDetectedMarkers(gray, corners,ids)
#    if ids == None:
#        print("No IDS found")
#    else:
#        print(ids)
#        for i in range(len(ids)):
#            phi = getQuadrant(corners[i])
#            print(ids[i],", ", phi,sep='')
#
#    if cv2.waitKey(1)& 0xFF == ord('q'):
#        break
#    rawCapture.truncate(0)


while True:
    try:
        actual_position_array = []
        desired_setpoint = 6 # to be returned from computer vision function
        writeNumber(desired_setpoint)  
        actual_position_array = readBlock()
        print(actual_position_array)
        lcd.clear()
        lcd.message = "target: " + str(desired_setpoint) + "\nactual: " + str(actual_position_array)
        
    except:
        print("I2C Error")
        lcd.clear()
        lcd.message = "I2C Error"
