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

actual_position_array = []

# Computer Vision Setup
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2


#Default setup for camera with resolution and arucoDict and Parameters for aruco detection
resolution = (640,480)
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParam = cv2.aruco.DetectorParameters_create()
pos = 0
def writeNumber(value):
    bus.write_byte(address, value)
    print(value)
    #bus.write_byte_data(address, 1, value)
    return

def readBlock():
    number = []
    while number == []:
        try:
            number = bus.read_i2c_block_data(address, 5, 2) # 2 is number of bytes in data, 5 is the sent offset
        except:
            continue
    print(number)
    return number

#Calculates the quadrant that the center of the marker is located in. 0 is top left, 1 is top right, etc.
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

# Camera calibration function. Generates images to test if there is good white balance.
def calibrateCamera(camera):
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

calibrateCamera(camera)

# Video capture to continuously detect markers
rawCapture = PiRGBArray(camera, size=resolution)
time.sleep(0.1)
pos = 0
i = 0
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

#   Try to send quadrant to arduino and get position back.
    try:
        writeNumber(pos)
        try: 
            actual_position_array = readBlock()
        except:
            x = 0
            
        # Gets counts from array in from arduino
        counts = int.from_bytes(actual_position_array, byteorder = 'big', signed = 'true')

#       Calculate radians of wheel. Could be degrees for readability.
        radians = (-counts / 3200 * 2 * 3.1415)
        target_radians = pos * 3.1415 / 2
        i = i + 1;
#       Updates every 10 iterations. Could be faster but code slows down a lot.
        if i % 10 == 0:
    
            lcd.clear()
            lcd.message = "target: " + str(target_radians) + "\nactual: " + str(radians)
    except:
        lcd.clear()
        lcd.message = "I2C Error 1"

#   Press q to quit
    if cv2.waitKey(1)& 0xFF == ord('q'):
        break

#   Necessary call. Deletes most recently viewed frame
    rawCapture.truncate(0)
