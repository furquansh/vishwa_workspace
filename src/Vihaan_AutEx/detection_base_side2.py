#!/usr/bin/env python
import rospy 
import cv2
import cv_bridge
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import sign_detector
import time 
from realsense_depth import *



class arrow_detection():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.image_callback)
        # global bridge
        # d435i depth image object
        # dc = DepthCamera()
        self.instruc_pub = rospy.Publisher('/motor_vals', Int32MultiArray, queue_size=10)
        self.left_cascade = cv2.CascadeClassifier('haar_trained_xml/left/cascade.xml')
        self.right_cascade = cv2.CascadeClassifier('haar_trained_xml/right/cascade.xml')
        self.rate = rospy.Rate(20)
        
        # point to find arrow dist using depth image 
        # point = (400, 300)
        # self.image_rs()

        self.flag = 0
  
    def pubstop(self):
        motor_vals_array = [0,0]
        data_to_send = Int32MultiArray()
        data_to_send.data = motor_vals_array
        self.instruc_pub.publish(data_to_send)
        self.flag = 1
        
    def cone_detect(self, frame):
        # convert the image to HSV because easier to represent color in
        # HSV as opposed to in BGR 
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of orange traffic cone color in HSV
        lower_orange1 = np.array([0, 135, 135])
        lower_orange2 = np.array([15, 255, 255])
        upper_orange1 = np.array([159, 135, 80])
        upper_orange2 = np.array([179, 255, 255])

        # threshold the HSV image to get only bright orange colors
        imgThreshLow = cv2.inRange(hsv_img, lower_orange1, lower_orange2)
        imgThreshHigh = cv2.inRange(hsv_img, upper_orange1, upper_orange2)

        # Bitwise-OR low and high threshes
        threshed_img = cv2.bitwise_or(imgThreshLow, imgThreshHigh)

        # smooth the image with erosion, dialation, and smooth gaussian
        # first create a kernel with standard size of 5x5 pixels
        kernel = np.ones((5,5),np.uint8)

        # get rid of small artifacts by eroding first and then dialating 
        threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations = 3)
        threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations = 2)

        # account for cones with reflective tape by dialating first to bridge the gap between one orange edge
        # and another and then erode to bring the traffic cone back to standard size
        smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations = 11)
        smoothed_img = cv2.erode(smoothed_img, kernel, iterations = 7)

        # detect all edges witin the image
        edges_img = cv2.Canny(smoothed_img, 100, 200)
        contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # set parameters for writing text and drawing lines
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        fontColor = (0, 0, 255)
        lineType = 2

        # analyze each contour and deterime if it is a triangle
        for cnt in contours:
            boundingRect = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.06 * cv2.arcLength(cnt, True), True)
            # if the contour is a triangle, draw a bounding box around it and tag a traffic_cone label to it
            if len(approx) == 3:
                x, y, w, h = cv2.boundingRect(approx)
                rect = (x, y, w, h)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                bottomLeftCornerOfText = (x, y)
                # comment this to optimize code speed. This is used for verifying cone detection
                cv2.putText(frame,'traffic_cone', 
                    bottomLeftCornerOfText, 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)
                
                return [1, x, y, w, h]


    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data)
        self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        self.gray = cv2.resize(self.gray, (640,480), interpolation=cv2.INTER_AREA)
        self.gray = cv2.medianBlur(self.gray, 5)
        kernel = np.ones((5,5),np.uint8)
        self.grayp = cv2.erode(self.gray, kernel, iterations = 3)
        self.grayp = cv2.dilate(self.grayp, kernel, iterations = 2)
        self.edges = cv2.Canny(self.gray,100,150,apertureSize = 3)

        # show what the image looks like after the application of previous functions
        cv2.imshow("current state", self.edges)
        # pause ke liye
        cv2.waitKey(0) 
        # original approach
        #perform HoughLines on the image
        # lines = cv2.HoughLines(edges,1,np.pi/180,30)
        # lines = cv2.HoughLinesP(self.edges,1,np.pi/180,70, minLineLength=150, maxLineGap=250)
        # print(type(lines))
        left_signs = sign_detector.classify_signs(self.edges, self.left_cascade)
        right_signs = sign_detector.classify_signs(self.edges, self.right_cascade)
        
        sign_detector.show_box(self.edges, left_signs)
        sign_detector.show_box(self.edges, right_signs)
        cone_detector=[]
        cone_detector=self.cone_detect(self.image)

        if len(right_signs):
            x,y,w,h = right_signs[0]
            distance = self.distance_to_camera(30, 610, w)-1
            self.pubctrl(0,distance)
            if(distance>0):
                print("right sign")
                print(distance)
        
        elif len(left_signs):
            print("left sign")
            x,y,w,h = left_signs[0]
            distance = self.distance_to_camera(30, 610, w)-1
            self.pubctrl(1,distance)
            print (distance)
        # elif(cone_detector is not None and cone_detector[0]):
        #     print("cone detected")
        #     w=cone_detector[3]
        #     distance1= self.distance_to_camera(75, 610, w)-1
        #     print(distance1)
        #     if(distance1<100):
        #         self.pubstop()
        else:
            print("Going straight...")
            self.pubctrl(2,0)

        # debug frame show
        # cv2.imshow("Frame", self.image)
        key = cv2.waitKey(1)
        self.rate.sleep()

    def distance_to_camera(self, knownWidth, focalLength, perWidth):
        return (knownWidth * focalLength) / perWidth

    def pubctrl(self, data, distance):
        templ=0
        tempr=0
        
        if(data==2):
            tempr=50
            templ=50
            
        if(1<distance<1500):
            rate2=rospy.Rate(3)
            rate2.sleep()
            if(data):
                tempr = 3
                templ = -3
            else:
                tempr = -3
                templ = 3
        
        # motor_vals_array = [self.motor_vals_vel_l, self.motor_vals_vel_r]
        motor_vals_array = [templ, tempr]
        data_to_send = Int32MultiArray()
        data_to_send.data = motor_vals_array
        self.instruc_pub.publish(data_to_send)
        rate = rospy.Rate(0.5)
        # increase speed here ^
        rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("autonomous")
    detection = arrow_detection()
    if detection.flag==0:
        rospy.spin()

    # dont shutdown here