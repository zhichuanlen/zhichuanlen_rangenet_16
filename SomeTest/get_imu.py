#保存机器人的xy路径

#rosrun imu_complementary_filter complementary_filter_node
#rosrun imu_to_odom imu_to_odom
#python get_imu.py
#
#


# !/usr/bin/env python

from cgi import FieldStorage
import imp
from time import time
import rospy
from sensor_msgs.msg import Imu
from  nav_msgs.msg  import Odometry
from scipy.spatial.transform import Rotation as R
import math
import matplotlib.pyplot as plt


First_time = True
time_last = 0.0
v_x = 0.0
v_y = 0.0
x = 0.0
y = 0.0
write_time = 0

O_x = []
O_y = []

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def to_euler_angles(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*z))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    angles['roll'] = r*180/math.pi
    angles['pitch'] = p*180/math.pi
    angles['yaw'] = y*180/math.pi

    return angles


# def imu_callback(data):
#     global First_time,time_last,v_x,v_y,x,y,O_x,O_y,write_time

#     write_time += 1
#     time_now = data.header.stamp
    
#     if(First_time):
#         time_last = time_now
#         First_time = False
#         print(time_last)
#     time_use = time_now - time_last
#     time_last = time_now
#     time_use_s = float(int(str(time_use)))/10e8
    
#     a_x = data.linear_acceleration.x
#     a_y = data.linear_acceleration.y
#     v_x = v_x + a_x*time_use_s
#     v_y = v_y + a_y*time_use_s
#     # print(v_y)
#     q = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
#     euler = quaternion2euler(q)
#     x = math.sin(euler[2])*v_x*time_use_s +x 
#     y = math.cos(euler[2])*v_x*time_use_s +y 
#     O_x.append(x)
#     O_y.append(y)
#     print(str(x) + "    "+str(y))
#     # print('\n')
#     if write_time%50 == 0:
#         with open("/home/ubuntu/zhichuanlen_rangenet_16/SomeTest/imu_xy.txt", "a") as file:
#             file.write(str(x)+' '+str(y)+ "\n")

def slam_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.z
    with open("/home/ubuntu/zhichuanlen_rangenet_16/SomeTest/slam_xy.txt", "a") as file:
          file.write(str(x)+' '+str(y)+ "\n")

def imu_callback(data):
    global write_time
    write_time += 1
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    if write_time%50 == 0:
        with open("/home/ubuntu/zhichuanlen_rangenet_16/SomeTest/imu_xy.txt", "a") as file:
            file.write(str(x)+' '+str(y)+ "\n")


def listener():
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber('/imu_raw', Imu, imu_callback)
    rospy.Subscriber('/aft_mapped_to_init', Odometry, slam_callback)
    rospy.Subscriber('/imu_odom', Odometry, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()