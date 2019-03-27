#!/usr/bin/env python


import rospy
import numpy as np
import random
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


pub = rospy.Publisher('skin_trigger', Bool, queue_size=1000)


def callback(data):
    #this function is called when the lisner is called and it also call the reconstruction
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    reconstruction(data)

def reconstruction(data):
    m= data.layout.dim[0].size     #getting array widht from the arduino publisher
    n= data.layout.dim[1].size     #getting array height from the arduino publisher

    arduino_data = data.data

    # recosntruction
    
    """ arduino_data = np.array(arduino_data)
    for i in range (len(arduino_data)):
        #print(i)
        arduino_data[i]= arduino_data[i] + round(random.uniform(0.1, 0.9),3) """
    trigger = bool(random.getrandbits(1))
    #reconstruction end here

    

    #print ("--------Start of Reconstruction data------")
    rospy.loginfo(trigger)
    #print ("---------End of Reconstruction data ------")
    pub.publish(trigger)
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('skin_trigger', anonymous=True)
    rospy.Subscriber('arduino_trigger', Float32MultiArray,  callback)

    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    listener()
