#!/usr/bin/env python


import rospy
import numpy as np
import random
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from time import sleep


from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

cloud_points = []
pub = rospy.Publisher('reconstruction', Float32MultiArray, queue_size=1000)
m=7 #have to create dynamically later
n=7 #have to create dynamically later

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    rviz_visualizer(data)

def rviz_visualizer(data):
    reconstructed_data = data.data
    #print(reconstructed_data)
    
   
    
    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #creat a 7 by 7 matrix
    pos_count=0
    for i in range(7):
        for j in range(7):
            map_array=[i-4 , j-4, round(reconstructed_data[pos_count],3)]
            #print(map_array)
            pos_count = pos_count+1
            cloud_points.append(map_array)
    print(cloud_points)



    #give time to roscore to make the connections
    #rospy.sleep(1.0)
    #cloud_points = [[1.0, 1.0, 3.0],[1.0, 4.0, 2.0],[4.0, 4.0, 3.0],[2.0, 2.0, 3.0],[4.0, 1.0, 1.0]]
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    #publish    
    
    #rospy.loginfo(scaled_polygon_pcl)
    pcl_pub.publish(scaled_polygon_pcl)
    del cloud_points[:]
    #sleep(0.1)


    #rospy.spin()
    #pub.publish(reconstruction_data)
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('reconstruction', Float32MultiArray,  callback)

    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    listener()
