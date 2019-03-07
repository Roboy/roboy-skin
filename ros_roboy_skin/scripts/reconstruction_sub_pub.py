#!/usr/bin/env python


import rospy
import numpy as np
import random
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


pub = rospy.Publisher('reconstruction', Float32MultiArray, queue_size=1000)


def callback(data):
    #this function is called when the lisner is called and it also call the reconstruction
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    reconstruction(data)

def reconstruction(data):
    m= data.layout.dim[0].size     #getting array widht from the arduino publisher
    n= data.layout.dim[1].size     #getting array height from the arduino publisher

    print("m and n")
    print(m)
    print(n)
    arduino_data = data.data

    # recosntruction
    
    arduino_data = np.array(arduino_data)
    for i in range (len(arduino_data)):
        #print(i)
        arduino_data[i]= arduino_data[i] + round(random.uniform(0.1, 0.9),3)
    
    #reconstruction end here

    # send message
    mat_size= m*n
    reconstruction_data = Float32MultiArray()
    reconstruction_data.layout.dim.append(MultiArrayDimension())
    reconstruction_data.layout.dim.append(MultiArrayDimension())
    reconstruction_data.layout.dim[0].label = "W"
    reconstruction_data.layout.dim[1].label = "H"
    reconstruction_data.layout.dim[0].size = m
    reconstruction_data.layout.dim[1].size = n
    reconstruction_data.layout.dim[0].stride = m * n
    reconstruction_data.layout.dim[1].stride = m
    reconstruction_data.layout.data_offset = 0
    reconstruction_data.data = arduino_data #[0]* mat_size
    
    # save a few dimensions:
    """ dstride0 = reconstruction_data.layout.dim[0].stride
    dstride1 = reconstruction_data.layout.dim[1].stride
    offset = reconstruction_data.layout.data_offset
    #put the datat back into matrix formate    
    for i in range(m):
        for j in range(n):
            
            reconstruction_data.data[offset + i + dstride1*j] = arduino_data[i][j] """

    #print ("--------Start of Reconstruction data------")
    rospy.loginfo(reconstruction_data)
    #print ("---------End of Reconstruction data ------")
    pub.publish(reconstruction_data)
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', Float32MultiArray,  callback)

    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    listener()
