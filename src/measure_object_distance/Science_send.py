import serial
import struct
import time
import rospy
from std_msgs.msg import Int32
class ScienceSend(): 
    def __init__(self):    
        rospy.init_node('sciencesend')

# Set the COM port based on your system (COMx for Windows, /dev/ttyUSB0 for Linux, /dev/cu.SLAB_USBtoUART for macOS)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=921600, timeout=1)
        rospy.Subscriber("science_joy", Int32, self.callback)
        self.rate = rospy.Rate(5)
        rospy.spin()



# def getvalue():
#     sendingdata =500
#     ser.write(sendingdata)
#     arduinodata = ser.readline().decode()
#     return arduinodata
    def send_integer(self, value):
        # Convert the integer to bytes
        self.data_to_send = value.to_bytes(4, byteorder='big')  # Assuming a 4-byte integer

        # Send the data
        self.serial_port.write(self.data_to_send)
        

# Example usage
    def callback(self, data):
     try:
        
                # Send an integer value (replace 42 with your desired integer)
                # print(data.data)
                self.send_integer(data.data)
                print("sent ")

                self.read = self.serial_port.readline().decode()
                print(self.read)
                
                # Wait for a short duration before sending the next value
                self.rate.sleep()

     except KeyboardInterrupt:
        # Close the serial port on program exit
        self.serial_port.close()

# while 1:
  
        
#         # print(getvalue())
#         # print()
#         a=int(1)
#         incomingdata = ser.write(b'9')
#         # print(type(incomingdata))
#         # print(incomingdata) 
#         read = ser.readline().decode()
#         print(type(read))
#         time.sleep(0.1)
#         print(read)
    
   
# def send_command(johnson, wormgear_pwm, servo_pwm, pumps):
#     command = struct.pack('i i i ?', johnson, wormgear_pwm, servo_pwm, pumps)
#     ser.write(command)

# try:
#     while True:
#         johnson = int(input("Enter Johnson value (0 for Stationary, 1 for UP, -1 for DOWN): "))
#         wormgear_pwm = int(input("Enter Wormgear PWM value: "))
#         servo_pwm = int(input("Enter Servo PWM value: "))
#         pumps = bool(int(input("Enter Pumps value (0 for OFF, 1 for ON): ")))

#         send_command(johnson, wormgear_pwm, servo_pwm, pumps)
#         time.sleep(5)

# except KeyboardInterrupt:
#     print("Program terminated by user")
# finally:
#     ser.close()




    
       
    
if __name__== '__main__':
    
    ScienceSend()