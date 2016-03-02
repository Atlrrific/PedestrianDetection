# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import RPi.GPIO as GPIO

from threading import Thread

from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# initialize the camera and grab a reference to the raw camera capture
resX = 240
resY = 180
camera = PiCamera()
camera.resolution = (resX,resY)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(resX, resY))

print(time.strftime("%H_%M_%S"))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(time.strftime("%H_%M_%S")+'.avi',fourcc, 20.0, (resX, resY))

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
detectFlag = 0
detectCounter = [0]
# allow the camera to warmup
time.sleep(0.1)

GPIO.setmode(GPIO.BOARD)

GPIO.setup(16, GPIO.OUT)

def classfier(testImage,threadNum,capTime, detectCounter):
    #print(threadNum,capTime)
    (rects, weights) = hog.detectMultiScale(testImage, winStride=(8, 8),
        padding=(8, 8), scale=1.1)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

	# draw the final bounding boxes
    # if(pick):
    for (xA, yA, xB, yB) in pick:
        print("Image detected")
        detectCounter[0] = 0
        cv2.rectangle(testImage, (xA, yA), (xB, yB), (0, 255, 0), 2)
    # print(pick,"\n");
    curTime = time.time()
    #print ("Total time from capture", curTime - capTime)
    out.write(testImage)
    cv2.imshow("After NMS", testImage)

# capture frames from the camera
i = 0
frameCount = 0
prevTime = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if (detectCounter[0] < 10):
        GPIO.output(16, GPIO.LOW)
        print ("Waiting ", detectCounter[0])
        detectCounter[0] += 1
    else:
        GPIO.output(16,GPIO.HIGH)
    image = frame.array
    captureTime = time.time()
    # print("FRAME Time", captureTime-prevTime)
    prevTime = captureTime

    # if frameCount == 0:
        # frameCount = 0
    #if i == 0:
    t1 = Thread(target = classfier, args = (image,i,captureTime,detectCounter))
    t1.start()
    threadPick = t1.join()


    # cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

     # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        cleanup_stop_thread();
        sys.exit()
        t1.stop()

        break
