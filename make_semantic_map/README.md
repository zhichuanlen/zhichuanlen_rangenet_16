## 从semantic数据集生产三维pcd地图和二维彩色栅格地图ppm

### build
mkdir build
cd build
cmake ..
make
# 运行
./save_semantic_map --b=/media/ubuntu/zhi_chuan_len-/lidar_data/my_kitti_bin_16/sequences/03/velodyne/ --p=../test


