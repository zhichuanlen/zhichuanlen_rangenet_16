# kitti_velodyne_bin_to_pcd

 - [Visual Odometry / SLAM Evaluation 2012](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)


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

./make_dataset --m=bin2pcd --b=/media/ubuntu/zhi_chuan_len-/lidar_data/test/bin_test/ --p=/media/ubuntu/zhi_chuan_len-/lidar_data/test/pcd_test/ --l=/media/ubuntu/zhi_chuan_len-/lidar_data/test/label_test/ --bo=/media/ubuntu/zhi_chuan_len-/lidar_data/test/bin_out_test/ --lo=/media/ubuntu/zhi_chuan_len-/lidar_data/test/label_out_test/
```
