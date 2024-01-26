#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class ScienceJoy():
    def __init__(self):
        self.flag = 0
        self.pub=rospy.Publisher('/science_joy',Int32MultiArray,queue_size = 10)
        self.wormgear1_pwm = 0
        self.wormgear2_pwm = 0
        self.johnson_pwm = 0
        self.servo_pwm_plate = 0
        self.servo_pwm_shake = 0
        self.pump1 = 0 
        self.pump2 = 0        # self.pub2=rospy.Publisher('wheel_angs',Int32MultiArray,queue_size = 10)

        

        self.sub = rospy.Subscriber("/joy", Joy, self.callback)
        



    def callback(self, data):
        
        if(data.buttons[1]== 1): #worm gear forward cmd 

               
                self.wormgear1_pwm = floor(data.buttons[1])

        if(data.buttons[1]== 0): #worm gear forward cmd 

               
                self.wormgear1_pwm = floor(data.buttons[0])
        if(data.buttons[1]== 1 and data.buttons[3]==1): #worm gear forward cmd 

               
                self.wormgear1_pwm = 2
        

       #  if(data.buttons[7]== -1 ): #worm gear forward cmd 

               
       #          self.wormgear1_pwm = floor(2.0)
       #  if(data.buttons[7]== 0 ): #worm gear forward cmd 

               
       #          self.wormgear1_pwm = floor(0.0)


       #  if(data.axes[4]<0): #worm gear backward cmd 
            
       #          self.wormgear1_pwm = 0*floor(100*(data.axes[4]))
           
        if(data.axes[1]>=0): #worm gear2 forward cmd 

               
                self.wormgear2_pwm = floor(100*(data.axes[1]))
              
        if(data.axes[1]<0): #worm gear2 backward cmd 
            
                self.wormgear2_pwm = 0*floor(100*(data.axes[1]))
               
        if(data.axes[6]==1): # johnson fwd cmd  
            
                self.johnson_pwm = floor(100*(data.axes[6]))
              
        if(data.axes[6]== -1): # johnson bwd cmd  
            
                self.johnson_pwm = 0*floor(100*(data.axes[6]))
               
        if(data.axes[6]==-0): # johnson stop cmd  
            
                self.johnson_pwm = floor(100*(data.axes[6]))
                
        if(data.buttons[5] == 1):  #servo movement 
               
               self.servo_pwm_plate = (150*data.buttons[5])
        else :
               self.servo_pwm_plate = 0
               
        if(data.buttons[4] == 1):   #servo movement 
               self.servo_pwm_shake = data.buttons[4]
        else :
               self.servo_pwm_shake = 0
               
        if(data.buttons[7] ==1):
               self.pump1 = data.buttons[1]
        else:
               self.pump1 = 0
               
        if(data.buttons[0] ==1):
               self.pump2 = data.buttons[0]
        else :
               self.pump2 = 0
               


        array = [self.wormgear1_pwm , self.wormgear2_pwm, self.johnson_pwm, self.servo_pwm_plate,self.servo_pwm_shake, 
                         self.pump1, self.pump2 ]
        
        array_to_send = Int32MultiArray()
        array_to_send.data = array
        self.pub.publish(array_to_send)
        print(array_to_send.data)
        print("published")

        # print (type(data.buttons[5])/)

        

        
               
        
         

        
            
            
        # elif(data.axes[3]>0): # left differential command

        #         self.wheel_1_r = -floor((100*data.axes[3]))
        #         self.wheel_2_l = -floor((100*data.axes[3]))
        #         self.wheel_3_l = floor((100*data.axes[3]))
        #         self.wheel_4_r = floor((100*data.axes[3]))

        # elif(data.axes[1]>0): # forward command
        #         self.wheel_1_r = floor(155*data.axes[1])
        #         self.wheel_2_l = floor(155*data.axes[1])
        #         self.wheel_3_l = floor(155*data.axes[1])
        #         self.wheel_4_r = floor(155*data.axes[1])

        # elif(data.axes[1]<0): # backward command
        #         self.wheel_1_r = floor(155*data.axes[1])
        #         self.wheel_2_l = floor(155*data.axes[1])
        #         self.wheel_3_l = floor(155*data.axes[1])
        #         self.wheel_4_r = floor(155*data.axes[1])

        # wheel_vels_arr = [self.wheel_1_r , self.wheel_2_l , self.wheel_3_l , self.wheel_4_r]
        # # ang_arr = [self.ang_wheel_1 , self.ang_wheel_2 , self.ang_wheel_3 , self.ang_wheel_4]
        # wheel_data_to_send = Int32()
        # # ang_data_to_send = Int32MultiArray()
        # wheel_data_to_send.data = wheel_vels_arr
        # # ang_data_to_send.data = ang_arr
        # self.pub1.publish(wheel_data_to_send)
        # # self.pub2.publish(ang_data_to_send)
        # print(str(wheel_data_to_send.data) + "--- wheels speed")
        # # print(str(ang_data_to_send.data) + "--- wheel angles")


    
def main():
    rospy.init_node('ScienceJoy')
    science = ScienceJoy()
    print("first loop looped")
    
    rospy.spin()
  
    

if __name__== '__main__':
    main()
    