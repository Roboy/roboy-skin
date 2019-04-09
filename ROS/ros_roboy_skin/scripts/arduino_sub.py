#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


started = False
pub = rospy.Publisher('reconstriction', Float32MultiArray, queue_size=1000)
m=7 #have to create dynamically later
n=7 #have to create dynamically later

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    reconstruction(data)

def reconstruction(data):
    arduino_data = data.data

    # recosntruction

    #just to test dummy reconstruction 
    #have to delete when the actual reconstruction is implemented
    arduino_data = np.array(arduino_data)
    arduino_data = arduino_data + 0.12
    
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
    


    print ("--------Start of Reconstruction data------")
    rospy.loginfo(reconstruction_data)
    print ("---------End of Reconstruction data ------")
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
