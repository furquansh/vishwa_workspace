#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import message_filters
import math


class Distance_detect:
    def __init__(self):
        rospy.init_node("Distance_Detect", anonymous=False)
        self.bridge = CvBridge()
        im_sub = message_filters.Subscriber('/camera/color/image_raw', Image,)
        dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 100, 0.01)
        self.timeSynchronizer.registerCallback(self.Detect_distance)
        print("first loop looped")
        rospy.spin()
        
       

    def Detect_distance(self, image_data, depth_data):
            
            if not isinstance(image_data, Image):return
            if not isinstance(depth_data, Image):return

            colour_frame = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough') 
    
            font = cv2.FONT_HERSHEY_DUPLEX
            colour_frame=cv2.flip(colour_frame,-1)
            depth_frame=cv2.flip(depth_frame,-1)
            
            hsv = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2HSV)
            lower_green = np.array([59,90,151])
            upper_green = np.array([83,255,255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            blur = cv2.GaussianBlur(mask,(9,9),0)

            def nothing(x):
                pass

            # cv2.namedWindow("colour_frame")
            # cv2.createTrackbar("quality","colour_frame",1,100, nothing)
            # quality = cv2.getTrackbarPos("quality", "colour_frame")
            
            # quality = quality/100
            quality = 0.01
            corners = cv2.goodFeaturesToTrack(blur,5, quality, 10)

            if corners is not None:
                corners = np.intp(corners)
               
                for i in corners:
                    x, y = i.ravel()
                    cv2.circle(colour_frame, (x, y), 3, 150, -1)
            else:
                print("No corners found.")

            if corners is not None and len(corners) >= 5:
                for i in corners[0]:
                    a0 = i[0]
                    b0 = i[1]
                for i in corners[1]:
                    a1 = i[0]
                    b1 = i[1]
                for i in corners[2]:
                    a2 = i[0]
                    b2 = i[1]
                for i in corners[3]:
                    a3 = i[0]
                    b3 = i[1]
                for i in corners[4]:
                    a4 = i[0]
                    b4 = i[1]

                    #### FIND THE MIDPOINT
                am= math.floor((a0+a1+a2+a3+a4)/5)
                bm= math.floor((b0+b1+b2+b3+b4)/5)  

                point = (am,bm)
                cv2.circle(colour_frame, point,4,(0,0,255) )
                distance = depth_frame[point[1], point[0]]
                print("Arrow Distance -->" + str(distance))
            
    

                contours, heirarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # print(len(contours))
                
                if len(contours)>0:
                    contours=sorted(contours, key=lambda x:cv2.contourArea(x),reverse=True)

                else :
                    print("no contours found")

                for c in contours:
                        # find minimum area
                    x,y,w,h = cv2.boundingRect(c)
                    (x,y),radius = cv2.minEnclosingCircle(c)
                    center = (int(x),int(y))
                    radius = int(radius)
                    cv2.circle(colour_frame,center,radius,(0,255,0),2)
                    cv2.circle(colour_frame,center,2,(0,255,0),2)
                    cv2.circle(colour_frame,(int(am),int(bm)),2,(0,255,0),2)

                cv2.line(colour_frame, center,(int(am),int(bm)),(255,0,0),1)
                cv2.line(colour_frame,center,(int(radius+x),int(y)),(255,0,0),1)

                atan=math.atan2(int(bm)-int(y),int(am)-int(x))
                angle=math.degrees(atan)
                print ('angle=', angle)
                if(angle >= -45 and angle < 45):
                    cv2.putText(colour_frame,'RIGHT',(10,85),font,3,(0,0,255))
                    print("RIGHT")
                elif(angle >=45 and angle < 135):
                    cv2.putText(colour_frame,'DOWN',(10,85),font,3,(0,0,255))
                    print("DOWN")
                elif(angle >= -180 and angle <=-135): 
                    cv2.putText(colour_frame,'LEFT',(10,85),font,3,(0,0,255))
                    print("LEFT")
                elif(angle >=135 and angle <=180):
                    cv2.putText(colour_frame,'LEFT',(10,85),font,3,(0,0,255))
                    print("LEFT")
                elif(angle > -135 and angle < -45):
                    cv2.putText(colour_frame,'UP',(10,85),font,3,(0,0,255))
                    print("UP")

            else:
                print("No corners found or not enough corners.")

            # cv2.imshow("Depth Frame ", depth_frame)
            cv2.imshow("Color Frame ", colour_frame)          
            # print("hello World")    
            cv2.waitKey(10) 
        
if __name__=='__main__':
    Distance_detect()