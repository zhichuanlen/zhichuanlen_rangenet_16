//pub_my_scan.cpp
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

std::string bin_path = "/media/ubuntu/zhi_chuan_len-/lidar_data/my_shu_bin_16_without_raw_z/sequences/00/velodyne/";
int count = 0;
void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & cloud_ros);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmsg_2_bin");
    //创建句柄
    ros::NodeHandle n;

    ros::Subscriber pointcloud_sub = n.subscribe ("/rslidar_points",1,pointcloudCallback);
    ros::spin();
    return 0;

}


void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & cloud_ros)
{
  std::stringstream streamss;
  std::string num;
  streamss<<std::setfill('0')<<std::setw(6)<<count;      //生成000001、000100这样的编号
  streamss>>num;

  std::string out_bin_path = bin_path + num + ".bin";
  // std::string out_bin_path = bin_path + std::to_string(count) + ".bin";
  count++;
  std::ofstream my_bin_File (out_bin_path.c_str(), std::ios::out | std::ios::binary);      //创建二进制输出
  pcl::PointCloud<pcl::PointXYZI>::Ptr pclcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_ros, *pclcloud);                                                  //从pointcloud2转换成pcl::PointXYZRGBA
  for (auto point : pclcloud->points)
    {
        
        float p_x,p_y,p_z,p_i;
        p_x = point.x;
        p_y = point.y;
        // p_z = point.z - 1.202;
        p_z = point.z;
        p_i = point.intensity/255;
        if(point.x < 30 && point.x > -30)
        //if(1)
        {
          my_bin_File.write((char*)&p_x, sizeof(float));
          my_bin_File.write((char*)&p_y, sizeof(float));
          my_bin_File.write((char*)&p_z, sizeof(float));
          my_bin_File.write((char*)&p_i, sizeof(float));
        }
        
    }
  std::cout<<"saved:"<<out_bin_path<<std::endl;
  my_bin_File.close();

}

