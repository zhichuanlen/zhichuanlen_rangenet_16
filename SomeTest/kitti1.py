#!/usr/bin/env python


import os
from data_utils import *
from publish_utils import *
from kitti_utils import  Calibration

DAtA_PATH = '/home/wsj/data/kitty/RawData/2011_09_26/2011_09_26_drive_0005_sync'

def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    R = np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2


if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitty_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam',Image,queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud',PointCloud2,queue_size=10)
    ego_pub = rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu',Imu,queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps',NavSatFix,queue_size=10)
    box3d_pub = rospy.Publisher('kitti_3d',MarkerArray,queue_size=10)
    #model_pub = rospy.Publisher('kitti_car_model',Marker,queue_size=10)
    bridge = CvBridge()
	
	
    rate = rospy.Rate(10)
    
    df_tracking = read_tracking('/home/wsj/data/kitty/training/label_02/0000.txt')
    
    calib = Calibration('/home/wsj/data/kitty/RawData/2011_09_26',from_video=True)
    
    while not rospy.is_shutdown():

        #使用np读取点云 
        point_cloud = read_point_cloud(os.path.join(DAtA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        #发布点云数据
        publish_point_cloud(pcl_pub,point_cloud)
        
        #发布ego_car
        publish_ego_car(ego_pub)
        
        #发布汽车模型
        #publish_car_model(model_pub)
        
        #读取imu和gps数据
        imu_data = read_imu(os.path.join(DAtA_PATH, 'oxts/data/%010d.txt'%frame))
        
        #发布imu数据
        publish_imu(imu_pub,imu_data)
        
        #发布gps数据
        publish_gps(gps_pub,imu_data)
        
        rospy.loginfo("published...")
        rate.sleep()
        frame += 1
        frame %= 154
        
        
       






