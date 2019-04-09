#!/usr/bin/python
# Software License Agreement (BSD License)

## Arduino publisher to publish to the reconstruction skin_trigger
## to the 'arduino_trigger' topic

import rospy
import numpy as np

import serial
import serial.tools.list_ports

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import random

ports = list(serial.tools.list_ports.comports())
port = None

for p in ports:
    print('Checking port %s / %s' % (p[0], p[1]))
    if "uino" in p[1].lower():  # Find "ardUINO" and "genUINO" boards
        port = p
        break

if port is None:
    print('Could not find a connected Arduino')
    exit(0)

print('Using the Arduino connected on:')
print(port[0] + ' / ' + port[1])

portPath = port[0]       # Must match value shown on Arduino IDE
baud = 1000000                    # Must match Arduino baud rate
timeout = 5                       # Seconds
max_num_readings = 16000
num_signals = 1
 
 
 
def create_serial_obj(portPath, baud_rate, tout):
    """
    Given the port path, baud rate, and timeout value, creates
    and returns a pyserial object.
    """
    return serial.Serial(portPath, baud_rate, timeout = tout)
    
def read_serial_line(serial):
    
    serial_data = serial.readline();
    
    return serial_data

print ("Creating serial object...")
serial_obj = create_serial_obj(portPath, baud, timeout)

def talker():
    rospy.init_node('skin_trigger', anonymous=True)
    pub = rospy.Publisher('arduino_trigger', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz

	
    while not rospy.is_shutdown():
        
        m = 0
        n = 0

        while (True):

            serial_data = read_serial_line(serial_obj)

            if "Snapshot" in serial_data:
                m = int(serial_data[-3])
                n = int(serial_data[-5])
                print(m, n)
                break

        while (True):
            serial_data = read_serial_line(serial_obj)
            row_data = []
            unclean_data = []
            clean_data = []

            first_unclean_data = []
            first_clean_data = []
            ultimate_data = []

            if "Snapshot" not in serial_data:

                first_unclean_data = (serial_data.split(','))
                first_unclean_data.pop()

                #         print(first_unclean_data)
                #         print("++++++++++++++++++++++++++++++++")
                for x in range(m):
                    #             print(x)
                    serial_data = read_serial_line(serial_obj)

                    #             print(serial_data)
                    #             print("----------------------------")
                    unclean_data = (serial_data.split(','))
                    unclean_data.pop()

                    if unclean_data:
                        clean_data.append(unclean_data)

                clean_data.insert(0, first_unclean_data)
                #make the string into float
                clean_data = [map(float,i) for i in clean_data]
                #put them into an array
                data_array = np.array(clean_data)
                
                mat_size= m*n
                send_data = Float32MultiArray()
                send_data.layout.dim.append(MultiArrayDimension())
                send_data.layout.dim.append(MultiArrayDimension())
                send_data.layout.dim[0].label = "LED"
                send_data.layout.dim[1].label = "SEN"
                send_data.layout.dim[0].size = m
                send_data.layout.dim[1].size = n
                send_data.layout.dim[0].stride = m * n
                send_data.layout.dim[1].stride = m
                send_data.layout.data_offset = 0
                send_data.data = [0]* mat_size
                
                # save a few dimensions:
                dstride0 = send_data.layout.dim[0].stride
                dstride1 = send_data.layout.dim[1].stride
                offset = send_data.layout.data_offset
                #put the datat back into matrix formate    
                for i in range(m):
                    for j in range(n):
                        
                        send_data.data[offset + i + dstride1*j] = data_array[i][j]
                        
                pub.publish(send_data)
                rospy.loginfo("Publishing..")
                
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
