import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import threading

# LCD setup
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()  # uses board.SCL and board.SDA
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

actual_position_array = []
rawCapture = PiRGBArray(camera, size=resolution)
time.sleep(0.1)
pos = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):        
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (corners,ids,rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParam)
    cv2.aruco.drawDetectedMarkers(gray, corners,ids)
    cv2.imshow('video',gray)

#   Dummy variable assignment if no markers are available. Could be a try-except statement.
#   Don't want to continue to next loop as we still want to update the lcd screen
    if ids == None:
        x = 0
#   Detect marker. Assumes first marker detected is marker we care about.
    else:        
        pos = getQuadrant(corners[0])

#   Press q to quit
    keyPress = cv2.waitkey(1)&0xFF
    if keyPress == ord('q'):
        rawCapture.truncate(0)
        break
    elif keyPress == ord('c'):
        camera.shutter_speed = camera.exposure_speed
        g = camera.awb_gains
        camera.exposure_mode = 'off'
        camera.awb_mode = 'off'
        camera.awb_gains = g
    elif keyPress == ord('v'):
        camera.exposure_mode = 'on'
        camera.awb_mode = 'on'
    #   Necessary call. Deletes most recently viewed frame
    rawCapture.truncate(0)
