import serial
serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
import time 
value = 50

def send_integer(value):
        # Convert the integer to bytes
        data_to_send = value.to_bytes(4, byteorder='big')  # Assuming a 4-byte integer

        # Send the data
        serial_port.write(data_to_send)


try:
        while True:
                # Send an integer value (replace 42 with your desired integer)
                
                send_integer(value)
                print("sent ")
                value = value +10

                read = serial_port.readline().decode()
                print(read)
                
                # Wait for a short duration before sending the next value
                time.sleep(0.1)

except KeyboardInterrupt:
        # Close the serial port on program exit
        serial_port.close()
