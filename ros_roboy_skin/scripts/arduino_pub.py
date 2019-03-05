#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

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
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
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
