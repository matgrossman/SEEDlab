
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt


#Default setup for camera with resolution and arucoDict and Parameters for aruco detection
resolution = (640,480)
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParam = cv2.aruco.DetectorParameters_create()


#Helper function to calculate angle based on linear interpretation of the center x position
def getPhi(corners):
    xAvg = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0])/4.0
    phi = (53.5/resolution[0])*xAvg -26.75
    return phi

#Saves image and opens it for viewing
def cvexercise_1(camera):
    fileName = input("File Name:")

    rawCapture = PiRGBArray(camera)

# allow the camera to warmup
    time.sleep(0.1)
# grab an image from the camera
    print("Capturing Image...")
    try:
      camera.capture(rawCapture, format="bgr")
      image = rawCapture.array
    except:
      print("Failed to capture")

    # save the image to the disk
    print("Saving image "+fileName)
    try:
      cv2.imwrite(fileName, image)
    except:
      print("Couldn't save "+fileName)
      pass
    img = cv2.imread(fileName,1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#Resizes image based on floor division of resolution
def cvexercise_2(camera):
    fileName = input("File Name:")
    rawCapture = PiRGBArray(camera)

# allow the camera to warmup
    time.sleep(0.1)
# grab an image from the camera
    print("Capturing Image...")
    try:
      camera.capture(rawCapture, format="bgr")
      image = rawCapture.array
    except:
      print("Failed to capture")

    # save the image to the disk
    print("Saving image "+fileName)
    try:
        cv2.imwrite(fileName, image)
    except:
      print("Couldn't save "+fileName)
      pass
    img = cv2.imread(fileName,1)
    height,width = img.shape[:2]
    img = cv2.resize(img,(width//2,height//2))
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#Saves image in grayscale using cvtColor command
def cvexercise_3(camera):
    fileName = input("File Name:")
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    print("Capturing Image...")
    try:
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
    except:
        print("failed to capture")
        pass
    try:
        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        cv2.imwrite(fileName, image)
    except:
      print("Couldn't save "+fileName)
      pass
    img = cv2.imread(fileName,1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    
#Loads in picture and detects markers in image
def cvexercise_4():
    fileName = input("File Name:")
    try:
        image = cv2.imread(fileName,0)
    except:
        print("Can't find image"+fileName)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = cv2.aruco.DetectorParameters_create()
    (corners,ids,rejected) = cv2.aruco.detectMarkers(image, dictionary=arucoDict,parameters=arucoParam)
    if ids == None:
        print("No markers detected")
    else:
        print(ids)
        
#Uses video to find markers. Outputs markers on screen using the drawDetectedMarkers command
#Prints angle to the terminal. 
def cvexercise_5(camera):
    rawCapture = PiRGBArray(camera, size=(640,480))
    time.sleep(0.1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (corners,ids,rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParam)
        cv2.aruco.drawDetectedMarkers(gray, corners,ids)
        if ids == None:
            print("No IDS found")
        else:
            print(ids)
            for i in range(len(ids)):
                phi = getPhi(corners[i])
                print(ids[i],", ", phi,sep='')
        cv2.imshow("video",gray)
        if cv2.waitKey(1)& 0xFF == ord('q'):
            break
        rawCapture.truncate(0)


#Marker generator to create aruco markers.
def generate_marker(fileName):
    image = np.zeros((200,200,1), dtype="uint8")
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    cv2.aruco.drawMarker(arucoDict,16,200,image,1)
    cv2.imwrite(fileName,image)
    cv2.imshow("tag",image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#different function calls
#cvexercise_1(camera)
#cvexercise_2(camera)
#cvexercise_3(camera)
#cvexercise_4(camera)
cvexercise_5(camera)

#generate_marker(fileName)
