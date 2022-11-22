
###                                                                                                                                make_dataset.cpp
# 作用

由semantic kitti的64线数据集降采样（四线之中取一线）

# Usage
```
Options
  --help : produce help message
  --b : bin file folder
  --p : pcd file folder
  --l : label file folder
  --bo : bin file out folder
  --lo : label file out folder
  --m : mode - bin2pcd, pcd2bin, make_dataset
```
## Example
```
./make_dataset --m=bin2pcd --b=/home/docker_share/sequences/00/velodyne/ --p=/home/docker_share/sequences/00_pcd

./make_dataset --m=make_dataset --b=/media/ubuntu/zhi_chuan_len-/kitti/dataset/sequences/00/velodyne/  --l=/media/ubuntu/zhi_chuan_len-/kitti/dataset/sequences/00/labels/ --bo=/media/ubuntu/zhi_chuan_len-/lidar_data/my_kitti_bin_16/sequences/00/velodyne/ --lo=/media/ubuntu/zhi_chuan_len-/lidar_data/my_kitti_bin_16/sequences/00/labels/
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

mkdir build
cd build
cmake ..
make


# kitti_velodyne_bin_to_pcd

 - [Visual Odometry / SLAM Evaluation 2012](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)