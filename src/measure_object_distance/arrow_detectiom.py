#### IMPORT MODULES
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import time
import datetime


def nothing(x):
    pass

#### SET TRACKBARS FOR CORNERS TO BE DETECTED 
# cv2.namedWindow("img")
# cv2.createTrackbar("quality","img",1,100,nothing)



#### INITIALIZE THE CAPTURE
cap=cv2.VideoCapture(2)
font = cv2.FONT_HERSHEY_DUPLEX


#### FLIP THE VIDEO SINCE THE PI CAMERA IS INVERTED    
    # img=cv2.flip(img,-1)
a=1
while (a==1):
#### EDGE DETECTION    
    # Convert BGR to HSV
    _,img= cap.read()
    
    # cv2.imshow("before hsv", img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_green = np.array([59,90,151])
    upper_green = np.array([83,255,255])

#     lower_green = np.array([100, 50, 50])  # Adjust these values as needed
#     upper_green  = np.array([140, 255, 255]) 
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    #Blurring the mask
    blur = cv2.GaussianBlur(mask,(9,9),0)
    # cv2.imshow("blur", blur)

# #### DETETCT THE CORNERS USING THE TRACKBARS
    cv2.namedWindow("img")
    cv2.createTrackbar("quality","img",1,100,nothing)
    quality = cv2.getTrackbarPos("quality", "img")
    
    quality = quality/100
    corners = cv2.goodFeaturesToTrack(blur, 5, quality, 10)

# # Check if corners is not None before converting
    if corners is not None:
        corners = np.intp(corners)
        

        # Plot the corners
        for i in corners:
            x, y = i.ravel()
            cv2.circle(img, (x, y), 3, 150, -1)
    else:
        # Handle the case where no corners are found
        print("No corners found.")


# #    cv2.imshow('img',img)

# #### GET THE CORNERS DATA FOR FINDING THE MIDPOINT OF THE ARROW
#     # Get the corners data for finding the midpoint of the arrow
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
        am=(a0+a1)/2
        bm=(b0+b1)/2
        # print(am,bm)
    

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
            cv2.circle(img,center,radius,(0,255,0),2)
            cv2.circle(img,center,2,(0,255,0),2)
            cv2.circle(img,(int(am),int(bm)),2,(0,255,0),2)

        #Drawing lines
        cv2.line(img, center,(int(am),int(bm)),(255,0,0),1)
        cv2.line(img,center,(int(radius+x),int(y)),(255,0,0),1)

        atan=math.atan2(int(bm)-int(y),int(am)-int(x))
        angle=math.degrees(atan)
        print ('angle=', angle)
        if(angle >= -45 and angle < 45):
            cv2.putText(img,'RIGHT',(10,85),font,1,(255,255,0))
            print("RIGHT")
        elif(angle >=45 and angle < 135):
            cv2.putText(img,'DOWN',(10,85),font,1,(255,255,0))
            print("DOWN")
        elif(angle >= -180 and angle <=-135): 
            cv2.putText(img,'LEFT',(10,85),font,1,(255,255,0))
            print("LEFT")
        elif(angle >=135 and angle <=180):
            cv2.putText(img,'LEFT',(10,85),font,1,(255,255,0))
            print("LEFT")
        elif(angle > -135 and angle < -45):
            cv2.putText(img,'UP',(10,85),font,1,(255,255,0))
            print("UP")

    else:
        # Handle the case where no corners are found
        print("No corners found or not enough corners.")

# #### FIND THE MIDPOINT
#     am=(a0+a1)/2
#     bm=(b0+b1)/2
#     print(am,bm)

# #### FIND THE CONTOURS
#     image, contours = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     contours, hier=sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)

# #### DRAW A CIRCLE ARROUND THE ARROW TO FIND THE ANGLE OF THE ARROW
#     for c in contours:
#         # find minimum area
#         x,y,w,h = cv2.boundingRect(c)
#         (x,y),radius = cv2.minEnclosingCircle(c)
#         center = (int(x),int(y))
#         radius = int(radius)
#         cv2.circle(img,center,radius,(0,255,0),2)
#         cv2.circle(img,center,2,(0,255,0),2)
#         cv2.circle(img,(int(am),int(bm)),2,(0,255,0),2)

#     #Drawing lines
#     cv2.line(img,center,(int(am),int(bm)),(255,0,0),1)
#     cv2.line(img,center,(int(radius+x),int(y)),(255,0,0),1)
    
#     #Angles
#     atan=math.atan2(int(bm)-int(y),int(am)-int(x))
#     angle=math.degrees(atan)
#     print ('angle=', angle)
#     if(angle >= -45 and angle < 45):
#         cv2.putText(img,'RIGHT',(10,85),font,1,(255,255,0))
#         print("RIGHT")
#     elif(angle >=45 and angle < 135):
#         cv2.putText(img,'DOWN',(10,85),font,1,(255,255,0))
#         print("DOWN")
#     elif(angle >= -180 and angle <=-135): 
#         cv2.putText(img,'LEFT',(10,85),font,1,(255,255,0))
#         print("LEFT")
#     elif(angle >=135 and angle <=180):
#         cv2.putText(img,'LEFT',(10,85),font,1,(255,255,0))
#         print("LEFT")
#     elif(angle > -135 and angle < -45):
#         cv2.putText(img,'UP',(10,85),font,1,(255,255,0))
#         print("UP")
    
#### SHOWING AND SAVING THE OUTPUT
    cv2.imshow('img',img)
#     b = cv2.resize(img, (1280,720), fx = 0, fy = 0, interpolation = cv2.INTER_CUBIC)
#     out.write(b)

# #### RECORD THE PROCESSING TIME OF THE PI AND SAVE IT AS A TEXT FILE
#     stop = datetime.datetime.now()
#     now = stop-start
#     outstring = str(now.total_seconds()) + '\n'    
#     print(outstring)
#     with open('ARV.txt','a') as myFile:
#         myFile.write(outstring)
    
    
    if cv2.waitKey(10) & 0xFF==ord('q'):
        break
cv2.destroyAllWindows()