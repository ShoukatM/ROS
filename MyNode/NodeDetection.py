#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from lgsvl_msgs.msg import Detection3DArray
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
import carla



lgstamp = 0
autostamp = 0
framenum = 0
framenum1 = 0
file_path = ""
client = carla.Client('localhost', 2000) 
world  = client.get_world()

os.system('./filepath.sh')
f_path = open('/home/autoware/autoware-contents/shared_dir/Node_ws/Reports/config.txt', 'r')
path = f_path.readline()
file_path = path.strip()
f_path.close()
#print("filepath:",file_path)
csv_fp = open(file_path + "/Objects_GTD_Perception.csv", "w")
csv_write = csv.writer(csv_fp)
field = ['sensor_name', 'frame_id', 'timestamp_sec' ,'timestamp_nanosec', 'available', 'verified', 'label', 'position_x', 'position_y',
                  'position_z', 'size_x', 'size_y', 'size_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                  'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
csv_write.writerow(field)
csv_fp.close()

csv_fp1 = open(file_path + "/Autoware_PerceptionData.csv", "w")
csv_auto_write = csv.writer(csv_fp1)
fieldAuto=["frame_id","timestamp_sec" , "timestamp_nanosec" , "available", "verified", "vehicle_label", "signal_label", "class_likelihood",
            "centroid_x", "centroid_y", "centroid_z", "size_x", "size_y", "size_z", 
            "corner_1_x", "corner_1_y", "corner_1_z", "corner_2_x","corner_2_y", "corner_2_z", "corner_3_x",   
            "corner_3_y", "corner_3_z", "corner_4_x", "corner_4_y", "corner_4_z", "orientation_x",    
            "orientation_y", "orientation_z",  "velocity", "heading", "heading_rate","value"]
csv_auto_write.writerow(fieldAuto)
csv_fp1.close()

def carla_callback(msg):
   global file_path
   global lgstamp
   global autostamp
   global autoware_boxes
   global framenum1   
 

   
   egoloc=msg.pose.pose.position.x
   #print("ego:",msg.pose.pose.position.x,msg.pose.pose.position.y)
   for npc in world.get_actors().filter('*vehicle*'):
      if len(world.get_actors().filter('*vehicle*')) > 1 :
       
        # Filter out the ego vehicle
        if npc.type_id != "vehicle.toyota.prius":
            #print("vehicle_id",npc.type_id)
            
            npcloc=npc.get_transform().location.x
            a=msg.pose.pose.position.x
            b=msg.pose.pose.position.y
            c=msg.pose.pose.position.z
            dist=npcloc - egoloc
            
            
            # Filter for the vehicles within 50m
            if dist > 20 and dist < 30:
               print("dis-->",dist)
               csv_fp = open(file_path +"/Objects_GTD_Perception.csv", "a")
               csv_odo_write = csv.writer(csv_fp)
   
               odom_data=["odo",msg.header.frame_id,msg.header.stamp.secs, msg.header.stamp.nsecs,"True","False",npc.type_id,
                         npc.get_transform().location.x,npc.get_transform().location.y,npc.get_transform().location.z,msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.position.z]
   
               csv_odo_write.writerow(odom_data)
               csv_fp.close()
              #print("npc_lable:",msg.header.stamp.secs,msg.header.stamp.nsecs,npc.type_id,npc.get_transform().location.x,npc.get_transform().location.y,npc.get_transform().location.z)
      else:
         print("NO NPC's Spawanned in City....")



   
   
   

def auto_callback(msg):
   #print("len:",len(msg.objects))
   global file_path
   global lgstamp
   global autostamp
   global autoware_boxes
   global framenum
   autoware_boxes = msg.objects
   c_autoware_boxes = 0
   #print("len:",len(msg.objects))
   if len(msg.objects) >  0:
          c_autoware_boxes = 0
   csv_fp1 = open(file_path +"/Autoware_PerceptionData.csv", "a")
   csv_auto_write = csv.writer(csv_fp1)
   
   while len(msg.objects) > 0:
       #print("call")
       data=[framenum, msg.objects[c_autoware_boxes].header.stamp.secs, msg.objects[c_autoware_boxes].header.stamp.nsecs, True, False,
       msg.objects[c_autoware_boxes].label,
       msg.objects[c_autoware_boxes].space_frame,'0',
       msg.objects[c_autoware_boxes].pose.position.x,
       msg.objects[c_autoware_boxes].pose.position.y,
       msg.objects[c_autoware_boxes].pose.position.z,
      
       msg.objects[c_autoware_boxes].dimensions.x,
       msg.objects[c_autoware_boxes].dimensions.y,
       msg.objects[c_autoware_boxes].dimensions.z,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[0].x,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[0].y,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[0].z,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[1].x,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[1].y,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[1].z,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[2].x,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[2].y,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[2].z,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[3].x,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[3].y,
       msg.objects[c_autoware_boxes].convex_hull.polygon.points[3].z,
       msg.objects[c_autoware_boxes].pose.orientation.x,
       msg.objects[c_autoware_boxes].pose.orientation.y,
       msg.objects[c_autoware_boxes].pose.orientation.z]
       csv_auto_write.writerow(data)
       if len(autoware_boxes)-1 == c_autoware_boxes :
                break;
       c_autoware_boxes = c_autoware_boxes + 1
   
   framenum = framenum+1    
   csv_fp1.close() 


def Node():
       # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
       print("Node Started....")
       rospy.init_node('listener', anonymous=True)
       rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry,carla_callback)
       rospy.init_node('listener', anonymous=True)
       rospy.Subscriber("/detection/lidar_detector/objects", DetectedObjectArray, auto_callback)
   
   
       # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()
   
if __name__ == '__main__':
          
           Node()
