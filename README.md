## 概述 ##
该python脚本用于生成kitti格式的文件用于点云标注，其中包含多个小工具：

    pcd2bin：点云pcd文件转为bin文件
    bin2pcd：点云bin文件转为pcd文件
    bag2pcd/bag2bin：bag转为pcd或bin文件
    match_pcd_img：点云pcd文件与图像文件匹配。

## 环境配置 ##
    Python 3.8.2

### 依赖 ###
    `$ pip install numpy==1.23.5`  
    `$ pip install argparse`
    `$ pip install rosbag`  
    `$ pip install rosbags`  
    `$ pip install rosnumpy`  

## Generate dataSets pipeline ##
### 1. bag slice  ###
选择需要标注的场景，如果bag较大，需要进行bag切分。
### 2. bag convert  ###
将ros2的bag转为ros1的bag

    rosbags-convert bag_path/rosbag2_2022_08_05-14_03_19.bag

### 3. bag2pcd bag2img  ###
将bag文件中的点云topic转为pcd文件，将相机topic转为png图片

### 4. ground calibration  ###
地面矫正

### 5. 点云拼接 ###
根据地面矫正矩阵和激光雷达外参进行点云拼接

### 6. 激光图像匹配 ###
为每一帧pcd文件匹配时间最近的图片

### 7. 生成calib文件
根据相机内外参生成calib文件