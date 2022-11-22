## 因为我的研究内容之一（低密度点云的语义提取），需要数据集，想将semetic_kitti 的 64线点云转换成16线,并且要在ROS框架下验证其他算法，遂整理这个仓库用来存储数据集的制作工具

semetic kitti 结构
dataset
    sequences
        00
              labels
              velodyne
              calib.txt
              poses.txt
              times.txt
        01
              labels
              velodyne
              calib.txt
              poses.txt
              times.txt
…
…

### build
bash build.sh

#                                                                                                                               make_dataset.cpp
由semantic kitti的64线数据集降采样（四线之中取一线）
![Image text]( https://github.com/zhichuanlen/zhichuanlen_rangenet_16/blob/main/make_dataset/images/64_16.jpg)

bash cov_kitti_to_16.sh "/media/ubuntu/zhi_chuan_len-/kitti/dataset/sequences/01" "/media/ubuntu/zhi_chuan_len-/lidar_data/get_kitti_bin_16/sequences/01/"


#                                                                                                                                      bin _pcd_cov.cpp:
代码来自zzy
pcd 和 bin 之间的相互转换

Options
  --help : produce help message
  --b : bin file folder
  --p : pcd file folder
  --m : mode - bin2pcd, pcd2bin
  
./build/make_dataset_bin_pcd_cov --m=bin2pcd --b=/home/docker_share/sequences/00/velodyne/ --p=/home/docker_share/sequences/00_pcd


 # 语义点云数据播放脚本
 python pub_semetic_pointcloud2.py '/media/ubuntu/zhi_chuan_len-/lidar_data/get_kitti_bin_16/sequences/01/'
 ![Image text]( https://github.com/zhichuanlen/zhichuanlen_rangenet_16/blob/main/make_dataset/images/rviz.jpg)

