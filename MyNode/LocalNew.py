#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from autoware_msgs.msg import DetectedObjectArray
import numpy as np # NumPy Python library
from scipy.spatial import ConvexHull
import cmath
from scipy.spatial import distance
import os
import time
import sys
from numpy import *
import json
import csv
file_path =" "

def set():
       global file_path
       os.system('./filepath.sh')
       f_path = open('./Reports/config.txt', 'r')
       path = f_path.readline()
       file_path = path.strip()
       f_path.close()
       print("filepath:",file_path)

       csv_fp = open(file_path + "/GNSS_ODOM_Localization.csv", "w")
       csv_write = csv.writer(csv_fp)
       field = ["frame_id", "child_frame_id", "timestamp_sec" ,"timestamp_nanosec", "position_x", 
         "position_y" , "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"]
       csv_write.writerow(field)
       csv_fp.close()

       csv_fp1 = open(file_path + "/NDT_Pose_Localization.csv", "w")
       csv_auto_write = csv.writer(csv_fp1)
       fieldAuto=["frame_id","timestamp_sec" , "timestamp_nanosec" ,"position_x",
           "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"]
       csv_auto_write.writerow(fieldAuto)
       csv_fp1.close()


def ndt_pose_callback(msg):
      
   csv_fp1 = open(file_path +"/NDT_Pose_Localization.csv", "a")
   csv_auto_write = csv.writer(csv_fp1)
   #print(msg.header.frame_id)
   print(msg.header.stamp.secs)
   #ndt_data=[msg.header.frame_id,msg.header.stamp.secs]
   
   ndt_data=[msg.header.frame_id, msg.header.stamp.secs, msg.header.stamp.nsecs, 
   msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,
   msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
   csv_auto_write.writerow(ndt_data)
   #print("type:",type(msg.pose.position.x))
   #print("type:",type(msg.pose.orientation.x))
   csv_fp1.close() 


def gnss_odom_callback(msg):

   csv_fp = open(file_path +"/GNSS_ODOM_Localization.csv", "a")
   csv_auto_write = csv.writer(csv_fp)
   
   
   odom_data=[msg.header.frame_id, msg.child_frame_id, msg.header.stamp.secs, msg.header.stamp.nsecs,
   msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.position.z,
   msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, 
         msg.pose.pose.orientation.w]
   csv_auto_write.writerow(odom_data)
   
   csv_fp.close() 

def Node():
       # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
       print("Node Started....")
       rospy.init_node('Node', anonymous=True)
       rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, gnss_odom_callback)
       rospy.init_node('Node', anonymous=True)
       rospy.Subscriber("/ndt_pose", PoseStamped, ndt_pose_callback)
   
   
       # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()


   
if __name__ == '__main__':
     try:
           set()
           Node()
     
