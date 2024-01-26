#!/usr/bin/env python

## Code for AutEx IRC 2024 
## 


import rospy
import cv2 as cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import message_filters
import sign_detector
from std_msgs.msg import Int32MultiArray

import time
import math

class autonomous():
    def __init__(self):
        rospy.init_node("autonomous")
        self.bridge = CvBridge()
        # im_sub = message_filters.Subscriber('/camera/color/image_raw', Image,)
        # dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        # self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 100, 0.01)
        # self.timeSynchronizer.registerCallback(self.image_callback1,)

        self.im_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback1)
        self.dep_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback2)

        self.left_cascade = cv2.CascadeClassifier('haar_trained_xml/left/cascade.xml')
        self.right_cascade = cv2.CascadeClassifier('haar_trained_xml/right/cascade.xml')
        self.motor_pub = rospy.Publisher('/motor_vals', Int32MultiArray, queue_size=10)

        self.midpoint = [320,240]
        self.distance = 0

        # self.rate = rospy.Rate(20)
        self.flag =  0
        self.test_flag = 1
        
        self.leftwheelforw = 0
        self.leftwheelback = 0
        self.rightwheelforw = 0
        self.rightwheelback = 0


    def set_this(self, lf=0, lb=0, rf=0, rb=0):
        self.leftwheelforw = lf 
        self.leftwheelback = lb
        self.rightwheelforw = rf 
        self.rightwheelback = rb

        return [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]

    def pub_current(self):
        # torq_array = [lf, lb, rf, rb]
        data_to_send = Int32MultiArray()
        data_to_send.data = [self.leftwheelforw, self.leftwheelback, self.rightwheelforw, self.rightwheelback]
        self.motor_pub.publish(data_to_send)

    def interpret_Speed(self, jack, distance =15):

        if (jack == 1):
            self.set_this(50, 50, 50, 50)
            return                                 #straight case
        elif(jack == 1) & (10<distance<(1300)):
            self.set_this()                         #stop case                         
        elif (jack == 2 and 10<distance<(1300)) :
            # 90 turn left until arrow is detected agin
            self.set_this(0,0,20,20)
            # green flag is set because after turning for a while the arrow will be out of sight
            #    and jack2 will become unset, hence no further scanning of arrows
        elif (jack == 3, 10<distance<=(1300)) :
            # 90 turn right until arrow is detected agin
            self.set_this(20,20,0,0)

        self.pub_current()
        start = time.time()
        while (1):
            if(time.time()-start >10):
                print("Stop complete")
                break
            # while (1):
            #     if(time.time()-start >10):
            #         print("fulfilled")
            #         break
            
            # publish


    def image_callback2(self, depth_data):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 
        self.distance = self.depth_image[((self.midpoint[0])), ((self.midpoint[1]))]
        print(str(self.distance)+"-- dist")


    def image_callback1(self, image_data):
        self.image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        # self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 
        self.image = cv2.resize(self.image, (640,480))

        ###reduce the field of view to remove potential noise from trees and rocks
        self.image2 = self.image[200:,20:]
        # self.image  = cv2.rectangle(self.image , (0,0), (640,200), (0,0,0), -1) 
        # cv2.imshow("cropped", self.image2)
        cv2.waitKey(100)
        self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        # self.gray = cv2.resize(self.gray, (440,440), interpolation=cv2.INTER_AREA)
        self.gray = cv2.medianBlur(self.gray, 5)
        kernel = np.ones((5,5),np.uint8)
        self.grayp = cv2.erode(self.gray, kernel, iterations = 3)
        self.grayp = cv2.dilate(self.grayp, kernel, iterations = 2)
        self.edges = cv2.Canny(self.gray,125,150,apertureSize = 3)
        

        cv2.imshow("current state", self.image)
        # cv2.waitKey(100) 

        left_signs = sign_detector.classify_signs(self.edges, self.left_cascade)
        right_signs = sign_detector.classify_signs(self.edges, self.right_cascade)


        # self.midpoint_left= sign_detector.show_box(self.image, left_signs)
        # self.midpoint_right= sign_detector.show_box(self.image , right_signs)

        # cv2.imshow("arrow detect", self.image)
        cv2.waitKey(10)

        if len(left_signs):
            print(left_signs)
            x1, y1, w1, h1 = left_signs
            self.midpoint = [(x1[0]+(x1[0]+w1[0]))/2, (y1[0]+(y1[0]+h1[0])/2)]
            self.midpoint[0] = int(self.midpoint[0])
            self.midpoint[1] = int(self.midpoint[1])
            print(self.midpoint)
            print("left arrow being detected")
            distance = self.distance
            # distance_left_arrow = self.depth_image[midpoint_left[0], midpoint_left[1]]
            # distance_left_arrow = distance_left_arrow/10
            
            print(distance)
            if(distance):
                self.interpret_Speed(2, distance)

        elif len(right_signs):
            print(right_signs)
            x2, y2, w2, h2 = right_signs
            
            self.midpoint = [((x2[0]+(x2[0]+w2[0]))/2), (y2[0]+(y2[0]+h2[0])/2)]
            self.midpoint[0] = int(self.midpoint[0])
            self.midpoint[1] = int(self.midpoint[1])
            
            print(self.midpoint)
            print("right arrow being detected")
            distance = self.distance
            print(distance)
            if(distance):
                self.interpret_Speed(3, distance)
        else :
            # go straight // change if not using that 90deg rule
            print("no arrow being detected")
            self.interpret_Speed(1)

        self.pub_current()


if __name__=='__main__':
    
    Autonomous = autonomous()

    rospy.spin()

