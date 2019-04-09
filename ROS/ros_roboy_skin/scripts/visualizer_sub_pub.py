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


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    rviz_visualizer(data)

def rviz_visualizer(data):
    m= data.layout.dim[0].size     #getting array widht from the reconstruction publisher
    n= data.layout.dim[1].size     #getting array height from the reconstruction publisher
    reconstructed_data = data.data
    #print(reconstructed_data)

    #normalized the data
    norm_reconstructed_data = [float(i)/sum(reconstructed_data) for i in reconstructed_data]
    
    pcl_pub = rospy.Publisher("/skin_visualizer", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #creat a m by n matrix
    pos_count=0
    for i in range(m):
        for j in range(n):
            #creating point cloud using the format [x position , y position , reconstructin data in z as color intensity]
            map_array=[i-4 , j-4, round(norm_reconstructed_data[pos_count],3)] 
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
    rospy.init_node('skin', anonymous=True)
    rospy.Subscriber('reconstruction', Float32MultiArray,  callback)

    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    listener()
