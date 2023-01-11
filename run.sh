#bagConvert
# rosbags-convert bag_path/rosbag2_2022_12_10-15_16_55/ \
# --include-topic /pc/lidar/top/pointcloud /pc/lidar/left/pointcloud /pc/lidar/right/pointcloud

#bag2pcd
python bagConvert.py --bag_path bag_path/ --mode bag2pcd --pcd_path top_pcd_path/ --lidar_topic /pc/lidar/top/pointcloud
python bagConvert.py --bag_path bag_path/ --mode bag2pcd --pcd_path left_pcd_path/ --lidar_topic /pc/lidar/left/pointcloud
python bagConvert.py --bag_path bag_path/ --mode bag2pcd --pcd_path right_pcd_path/ --lidar_topic /pc/lidar/right/pointcloud

#pc_concat
python pc_concat.py --top_pcd_path top_pcd_path/ --left_pcd_path left_pcd_path/ --right_pcd_path right_pcd_path/

#ground_calibration
