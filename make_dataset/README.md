
# 因为我的研究内容之一（低密度点云的语义提取），需要数据集，所以将semetic_kitti 的 64线点云转换成16线,并且要在ROS框架下验证其他算法，遂整理这个仓库用来存储数据集的制作工具

###                                                                                                                                make_dataset.cpp
# 作用

由semantic kitti的64线数据集降采样（四线之中取一线）



## Example
```
bash cov_kitti_to_16.sh "/media/ubuntu/zhi_chuan_len-/kitti/dataset/sequences/01" "/media/ubuntu/zhi_chuan_len-/lidar_data/get_kitti_bin_16/sequences/01/"
```
# #############################################################################################

###                                                                                                                                      bin _pcd_cov.cpp:

# 作用

pcd 和 bin 之间的相互转换

# Usage
```
Options
  --help : produce help message
  --b : bin file folder
  --p : pcd file folder
  --m : mode - bin2pcd, pcd2bin
```

## Example
```
./binpcd --m=bin2pcd --b=/home/docker_share/sequences/00/velodyne/ --p=/home/docker_share/sequences/00_pcd
```

# #############################################################################################

# build

bash build.sh

 # 语义点云数据播放脚本
 python pub_semetic_pointcloud2.py '/media/ubuntu/zhi_chuan_len-/lidar_data/get_kitti_bin_16/sequences/01/'