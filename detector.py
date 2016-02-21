# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
from threading import Thread

from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 3
rawCapture = PiRGBArray(camera, size=(320, 240))

# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--images", required=True, help="path to images directory")
# args = vars(ap.parse_args())

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


# allow the camera to warmup
time.sleep(0.1)



def classfier(testImage,threadNum,capTime):
    print("\n\n\n\n",threadNum,capTime)
    (rects, weights) = hog.detectMultiScale(testImage, winStride=(4, 4),
        padding=(8, 8), scale=1.05)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

	# draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(testImage, (xA, yA), (xB, yB), (0, 255, 0), 2)

    curTime = time.time()
    print ("Total time from capture", curTime - capTime,"\n\n\n")
    if(weights):
        print(weights)
        cv2.imshow("After NMS", testImage)

# capture frames from the camera
i = 0
frameCount = 0
prevTime = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    # image2 = cv2.CreateImage(image)
    # image = imutils.resize(image, width=min(400, image.shape[1]))
    # orig = image.copy()
    # show the frame
    captureTime = time.time()
    # print("FRAME Time", captureTime-prevTime)
    prevTime = captureTime

    if frameCount == 3:
        frameCount = 0
        if i == 1:
            t1 = Thread(target = classfier, args = (image,1,captureTime,))
            t1.start()
        if i == 2:
            t2 = Thread(target = classfier, args = (image,2,captureTime,))
            t2.start()
        if i == 3:
            t3 = Thread(target = classfier, args = (image,3,captureTime,))
            t3.start()
        if i == 4:
            t4 = Thread(target = classfier, args = (image,4,captureTime,))
            t4.start()

        i += 1
        if i > 4:
            i = 1

    frameCount+=1

    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF


    # detect people in the image

    #
    # print(weights)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

     # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        cleanup_stop_thread();
        sys.exit()
        t1.stop()
        t2.stop()
        t3.stop()
        t4.stop()


        break



# loop over the image paths
# for imagePath in paths.list_images('./images'):
#     # load the image and resize it to (1) reduce detection time
#     # and (2) improve detection accuracy
#     image = cv2.imread(imagePath)
#     image = imutils.resize(image, width=min(400, image.shape[1]))
#     orig = image.copy()
#
#     # detect people in the image
#     (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
#         padding=(8, 8), scale=1.05)
#
#     print(weights)
    # draw the original bounding boxes
    # for (x, y, w, h) in rects:
    #     cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
    #
    # # apply non-maxima suppression to the bounding boxes using a
    # # fairly large overlap threshold to try to maintain overlapping
    # # boxes that are still people
    # rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    # pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    #
    # # draw the final bounding boxes
    # for (xA, yA, xB, yB) in pick:
    #     cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
    #
    # # show some information on the number of bounding boxes
    # filename = imagePath[imagePath.rfind("/") + 1:]
    # print("[INFO] {}: {} original boxes, {} after suppression".format(
    #     filename, len(rects), len(pick)))

    # show the output images
    # cv2.imshow("Before NMS", orig)
    # cv2.imshow("After NMS", image)
    # cv2.waitKey(0)
