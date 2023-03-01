#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import Image,PointCloud2,Imu,NavSatFix
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
import tf

FRAME_ID = 'map'
DETECTION_COLOR_DICT = {'Car':(255,255,0),'Pedestrian':(0,226,255),'Cyclist':(141,40,255)}
LIFETIME = 0.6

# connect vertic
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]] # lower face
LINES+= [[4, 5], [5, 6], [6, 7], [7, 4]] # upper face
LINES+= [[4, 0], [5, 1], [6, 2], [7, 3]] # connect lower face and upper face
LINES+= [[4, 1], [5, 0]] # front face and draw x


    
def publish_point_cloud(pcl_pub,point_cloud):
    header=Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
        
    pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3]))
    

    
def publish_ego_car(ego_car_pub):

    marker_array = MarkerArray()
    
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    
    marker.id = 0
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP
    
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2
    
    
    marker.points = []
    marker.points.append(Point(10,-10,0))
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,10,0))
    
    marker_array.markers.append(marker)
    
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()
        
    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti_turtorial/Audi R8/Models/Audi R8.dae"
        
        
    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73
        
        
    q = tf.transformations.quaternion_from_euler(0,0,-300);
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]
        
        
    mesh_marker.color.r=1.0
    mesh_marker.color.g=1.0
    mesh_marker.color.b=1.0
    mesh_marker.color.a=1.0
        
    mesh_marker.scale.x = 0.7
    mesh_marker.scale.y = 0.7
    mesh_marker.scale.z = 0.7
    
    marker_array.markers.append(mesh_marker)
    
    
    ego_car_pub.publish(marker_array)
    
    
    
"""    
def publish_car_model(model_pub):
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()
        
    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti_turtorial/Audi R8/Models/Audi R8.dae"
        
        
    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73
        
        
    q = tf.transformations.quaternion_from_euler(0,0,-300);
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]
        
        
    mesh_marker.color.r=1.0
    mesh_marker.color.g=1.0
    mesh_marker.color.b=1.0
    mesh_marker.color.a=1.0
        
    mesh_marker.scale.x = 0.7
    mesh_marker.scale.y = 0.7
    mesh_marker.scale.z = 0.7
        
    model_pub.publish(mesh_marker)
        
   """     
        
        

def publish_imu(imu_pub,imu_data):
    imu = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()
    
    #设置旋转量
    q = tf.transformations.quaternion_from_euler(float(imu_data.roll),float(imu_data.pitch),float(imu_data.yaw));
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    
    #设置线性加速度
    imu.linear_acceleration.x = imu_data.af
    imu.linear_acceleration.y = imu_data.al
    imu.linear_acceleration.z = imu_data.au
    
    #设置角加速度
    imu.angular_velocity.x = imu_data.wf
    imu.angular_velocity.y = imu_data.wl
    imu.angular_velocity.z = imu_data.wu
    
    imu_pub.publish(imu)
    
    
    
def publish_gps(gps_pub,imu_data):
    gps = NavSatFix()
    gps.header.frame_id = FRAME_ID
    gps.header.stamp = rospy.Time.now()
    
    #gps经度、纬度、海拔高度
    gps.latitude = imu_data.lat
    gps.longitude = imu_data.lon
    gps.altitude = imu_data.alt
    
    gps_pub.publish(gps)



    
