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
    serial_data = serial_data.decode("utf-8") #ser.readline returns a binary, convert to string
    #print (serial_data)
    return serial_data

print ("Creating serial object...")
serial_obj = create_serial_obj(portPath, baud, timeout)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
	
	#ma = np.random.random((10,10))
    
	
    while not rospy.is_shutdown():
        serial_data = read_serial_line(serial_obj)
	#LED = "LED-%s" % np.random.random((10,10))
	#SEN = "SEN-%s" % np.random.random((10,10))
	#DATA = LED + "\n" + SEN
        rospy.loginfo(serial_data)
        pub.publish(serial_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
