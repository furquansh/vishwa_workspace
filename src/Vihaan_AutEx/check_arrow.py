import cv2

cam = cv2.VideoCapture(0)
import rospy
import cv2 as cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import message_filters
import sign_detector
from std_msgs.msg import Int32MultiArray
bridge = CvBridge()
import numpy as np
import time

while True:
    check, frame = cam.read()
    # pts1 = np.float32([[0, 260], [640, 260],
    #                    [0, 400], [640, 400]])
    # pts2 = np.float32([[0, 0], [400, 0],
    #                    [0, 640], [400, 640]])
    
    # matrix = cv2.getPerspectiveTransform(pts1, pts2)
    # result = cv2.warpPerspective(frame, matrix, (640, 480))
    frame  = cv2.rectangle(frame, (0,0), (640,200), (0,0,0), -1)
    
    left_cascade = cv2.CascadeClassifier('haar_trained_xml/left/cascade.xml')
    right_cascade = cv2.CascadeClassifier('haar_trained_xml/right/cascade.xml')
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (640,480), interpolation=cv2.INTER_AREA)
    gray = cv2.medianBlur(gray, 5)
    kernel = np.ones((5,5),np.uint8)
    grayp = cv2.erode(gray, kernel, iterations = 3)
    grayp = cv2.dilate(grayp, kernel, iterations = 2)
    edges = cv2.Canny(gray,100,150,apertureSize = 3)

    left_signs = sign_detector.classify_signs(edges, left_cascade)
    right_signs = sign_detector.classify_signs(edges, right_cascade)
    print(type(left_signs))
    if len(left_signs):
        print (str(left_signs) + "-- left arrow")
    elif len(right_signs):
        print (str(right_signs) + "-- right arrow")

    cv2.imshow('video', edges)
    cv2.imshow("hi", frame)

    key = cv2.waitKey(10)
    if key == 27:
        break

cam.release()
cv2.destroyAllWindows()