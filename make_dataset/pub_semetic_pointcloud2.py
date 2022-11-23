#coding=utf-8
#!/usr/bin/env python2
import rospy
import struct
import time,os,sys
import numpy as np
import os
import threading
import tf
from threading import Lock, RLock
from tqdm import tqdm
from pyquaternion import Quaternion
from std_msgs.msg import Char,Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def label_to_color(one_label): #传入一个预测的类别代号，返回一个特定的rgb值
    dict_color = {
                                0 : [0, 0, 0],
                                1 : [0, 0, 255],
                                10: [245, 150, 100],
                                11: [245, 230, 100],
                                13: [250, 80, 100],
                                15: [150, 60, 30],
                                16: [255, 0, 0],
                                18: [180, 30, 80],
                                20: [255, 0, 0],
                                30: [30, 30, 255],
                                31: [200, 40, 255],
                                32: [90, 30, 150],
                                40: [255, 0, 255],
                                44: [255, 150, 255],
                                48: [75, 0, 75],
                                49: [75, 0, 175],
                                50: [110, 200, 255],
                                51: [50, 120, 255],
                                52: [0, 150, 255],
                                60: [170, 255, 150],
                                70: [0, 175, 0],
                                71: [0, 60, 135],
                                72: [80, 240, 150],
                                80: [150, 240, 255],
                                81: [0, 0, 255],
                                99: [255, 255, 50],
                                252: [245, 150, 100],
                                256: [255, 0, 0],
                                253: [200, 40, 255],
                                254: [30, 30, 255],
                                255: [90, 30, 150],
                                257: [250, 80, 100],
                                258: [180, 30, 80],
                                259: [255, 0, 0],
                                299: [255,255,255]}
    x = one_label
    x = int(x)
    if dict_color.__contains__(x):
        r = dict_color[x][2]
        g = dict_color[x][1]
        b = dict_color[x][0]
        a = 255
    else:
        r = 255
        g = 255
        b = 255
        a = 255
    rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]  #编码rgb打包成一个变量
    return rgba

def pub_pointcloud(data_path,label_path,pub_time):
    DATA_PATH = data_path
    label = np.fromfile(label_path, dtype=np.uint32)
    # print('point num ={label_size}'.format(label_size=label.size))
    DATA_LABEL = label
    point_cloud=np.fromfile(os.path.join(DATA_PATH),dtype=np.float32).reshape(-1,4)  #根据传来的地址吧bin文件转化成np.array   转换为nx4的矩阵
    #包含xyz坐标值，包含反射率，但是反射率最后被replace成了rgb
    header=Header() #定义pcl2消息的头部
    header.stamp=rospy.Time.now() #打上ros时间戳
    header.frame_id='rslidar' #方便调试，实际运行应改为Lidar的frame

    fields=[PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),]
    point_cloud=np.around(point_cloud,3)  #精确到小数点后三位
    point_cloud=point_cloud.tolist()      #从np.array转到list
    for h in range(min(len(DATA_LABEL),len(point_cloud))):
        point_cloud[h][3] = label_to_color(DATA_LABEL[h])    #赋予颜色，每一个数据由XYZ 和经过打包后的RGB信息组成。
        #打包RGB代码为 rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #其中rgba的取值都是0到255
    pcl2=point_cloud2.create_cloud(header,fields,point_cloud)  #组合编码成pcl2格式
    while(time.time() < pub_time):
        time.sleep(0.01)
    pub.publish(pcl2) #发布topic
    
def get_odom_data(odom_file_path):
    q_list = []
    groundtruth_list = []
    with open(odom_file_path) as f:
        for line in f.readlines():
            line = line.split(' ')
            # 将元素由字符串转为float
            line = list(map(float, line))
            # 向量转矩阵
            line = np.array(line)
            line.resize(3, 4)
            # data3_3 = line[:, :3]
            # q = Quaternion(matrix=data3_3)
            add = np.array([0,0,0,1])
            q = tf.transformations.quaternion_from_matrix(np.r_[line,[add]])
            q_list.append(q)
            groundtruth_list.append([line[0, 3], line[2, 3]])
        # 最后得到两个numpy矩阵，dataset是存放所有真值的矩阵，groundtruth是存放xy真值的矩阵
        q_list = np.array(q_list)
        groundtruth_list = np.array(groundtruth_list)
    return q_list , groundtruth_list

def get_time_data_list(time_file_path):
    with open(time_file_path) as f:
         return f.readlines()
    
def pub_odom(xy, q):
    send_Od_data=Odometry()
    send_tf_data = TransformStamped() 
    # 发布tf
    send_tf_data.header.stamp = rospy.Time.now()
    send_tf_data.header.frame_id = "odom"
    send_tf_data.child_frame_id = "base_link"

    send_tf_data.transform.translation.x = xy[0]
    send_tf_data.transform.translation.y = xy[1]
    send_tf_data.transform.translation.z = 0

    send_tf_data.transform.rotation.w = q[3]
    send_tf_data.transform.rotation.x = q[0]
    send_tf_data.transform.rotation.y = q[1]
    send_tf_data.transform.rotation.z = q[2]
    
    tf_pub.sendTransformMessage(send_tf_data)

    # 发布里程计信息
    send_Od_data.header.stamp = rospy.Time.now()
    send_Od_data.header.frame_id = "odom"
    send_Od_data.child_frame_id = "base_link"

    send_Od_data.pose.pose.position.x = xy[0]
    send_Od_data.pose.pose.position.y = xy[1]
    send_Od_data.pose.pose.position.z = 0

    send_Od_data.pose.pose.orientation.x = q[0]
    send_Od_data.pose.pose.orientation.y = q[1]
    send_Od_data.pose.pose.orientation.z = q[2]
    send_Od_data.pose.pose.orientation.w = q[3]
    
    odom_pub.publish(send_Od_data)
    
def R_t_2_q(R_t):
    matrix = []
    for l in R_t:
        matrix.append(l[0:3])
    matrix=np.array(matrix)
    q = Quaternion(matrix=matrix)
    return q

def show_odom_path():
    import matplotlib.pyplot as plt
    x_data = []
    y_data = []
#     print(len(dataset))
    for i in range(len(groundtruth)):
        x_data.append(float(groundtruth[i][0]))
        y_data.append(float(groundtruth[i][1]))
    # 绘制
    lock = Lock()
    with lock:
        plt.plot(x_data,y_data)
        plt.show()

def main_pub():
    time_begin = time.time()
    time_cost_sum = 0
    for i in tqdm(range(len(bin_files))):
        t_in = time.time()
        time_list = get_time_data_list(time_file_path)
        pub_time = time_begin + float(time_list[i])
        pub_pointcloud(bin_file_path+str(bin_files[i]),label_file_path+str(label_files[i]),pub_time)
        pub_odom(groundtruth[i], q_list[i])
            
        t_out = time.time()
        used_time = t_out-t_in
        time_cost_sum += used_time
    print("共发布"+str(len(bin_files))+"帧消息,平均帧率为"+str(1/(time_cost_sum/len(bin_files))))


if __name__ == "__main__":

    rospy.init_node('kitti_node', anonymous=True)#创建ros_node
    pub=rospy.Publisher('rslidar',PointCloud2,queue_size=10)#点云的publisher
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=100)  # 里程计publisher
    tf_pub = tf.TransformBroadcaster()  # TF变换publisher

    data_path  = sys.argv[1]
    bin_file_path   = data_path + 'velodyne/'
    label_file_path = data_path + 'labels/'
    odom_file_path  = data_path + 'poses.txt'
    time_file_path  = data_path + 'times.txt'


    bin_files = sorted(os.listdir(bin_file_path))
    label_files = sorted(os.listdir(label_file_path))

    q_list, groundtruth = get_odom_data(odom_file_path)
    pub_t = threading.Thread(target = main_pub)
    pub_t.start()
    show_odom_path()