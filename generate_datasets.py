"""
Date:2023.02.09
author: zhaobingzhen@tongxin.cn
describe: this script is used for generate kitti-format folder tree
"""

import os
import argparse
from bagConvert import bagConvert
from match_pcd_img import match
from pc_concat import concat_calib
from ground_calibration import ground_calib_func
from write_calib import writeCalib

def main(args):
    
    if dataSets == 'kitti':
        print("*********** Start to generate kitti datasets***************")
        folder_name = ['calib/',  'image_2/', 'label_2/', 'velodyne/']
        for i in folder_name:
            if not os.path.exists('kitti/training/' + i):
                os.makedirs('kitti/training/' + i)
    
    if dataSets == 'SUSTechPoints':
        print("*********** Start to generate SUSTechPoints datasets***************")
        folder_name = ['calib/',  'camera/', 'label/', 'lidar/']
        for i in folder_name:
            if i == 'camera/':
                if not os.path.exists('wit/' + i):
                    os.makedirs('wit/camera/front/')
                    os.makedirs('wit/camera/left/')
                    os.makedirs('wit/camera/right/')
            else:
                if not os.path.exists('wit/' + i):
                    os.makedirs('wit/' + i)
    
    print("************ Start to process the bagfile*****************")
    bagConvert(args,lidar_topics,camera_topics)
    
    if not ground_calib:
        print("**************Start to ground calibration***************")
        concat_calib(args,ground_calib,mat_file,interval)
        pcd_file = os.path.join('kitti/training/velodyne/', os.listdir('kitti/training/velodyne/')[0])
        ground_calib_func(pcd_file,mat_file)
   
    print("**************Start to concatenate PointCloud**************")
    concat_calib(args,ground_calib,mat_file,interval)
    
    if dataSets == 'kitti':
        lidar_path = 'kitti/training/velodyne/'
        dst_path = 'kitti/training/image_2/'
        calib_path = 'kitti/training/calib/'
    else:
        lidar_path = 'wit/lidar'
        dst_path = 'wit/camera/front/'
        calib_path ='wit/calib/'
    
    print("***************Start to match imge and pcd***************")
    for camera_topic in camera_topics:
        if camera_topic.split('/')[1] == 'camera0':
            img_path = camera_topic.split('/')[1] + '_img/'
            dst_path = 'wit/camera/front/'
            match(img_path,lidar_path,dst_path,tolerate_offset = 200)
        if camera_topic.split('/')[1] == 'camera4':
            img_path = camera_topic.split('/')[1] + '_img/'
            dst_path = 'wit/camera/right'
            match(img_path,lidar_path,dst_path,tolerate_offset = 200)
        if camera_topic.split('/')[1] == 'camera5':
            img_path = camera_topic.split('/')[1] + '_img/'
            dst_path = 'wit/camera/left/'
            match(img_path,lidar_path,dst_path,tolerate_offset = 200)
    #calib
    print("*****************Start to generate calib ******************")
    for file in os.listdir('wit/lidar'):
        fileName = file.replace('.pcd','.txt')
        writeCalib(calib_path+ fileName)

if __name__ == '__main__':
    parser =  argparse.ArgumentParser(description="Generate kitti-format for annotate")
    parser.add_argument("--bag_path",help='.bag file path.',type=str,default="bag_path/")
    parser.add_argument("--mode",help='.bag to .pcd or .bag to .bin',type=str,default="bag2pcd")
    parser.add_argument("--img_path",help="save img path",type=str,default="")
    parser.add_argument("--top_pcd_path",help="The top lidar pcd file path",type=str,default="top_pcd/")
    parser.add_argument("--right_pcd_path",help="The right lidar pcd file path",type=str,default="right_pcd/")
    parser.add_argument("--left_pcd_path",help="The left lidar pcd file path",type=str,default="left_pcd/")
    parser.add_argument("--concat_pcd_path",help="output concat_pc pcd file path",type=str,default="wit/lidar/")
    args = parser.parse_args()
    
    interval = 5 #抽帧
    lidar_topics = [
        "/pc/lidar/top/pointcloud",
        "/pc/lidar/left/pointcloud",
        "/pc/lidar/right/pointcloud"
    ]
    camera_topics = [
        "/camera0/image_raw/compressed",
        "/camera4/image_raw/compressed",
        "/camera5/image_raw/compressed"
    ]
    mat_file = 'ground_calib_mat.txt' #地面矫正矩阵
    # dataSets = 'kitti'
    dataSets = 'SUSTechPoints'
    if os.path.exists("ground.ply")  and os.path.exists("ground_calib_mat.txt"):
        ground_calib = True
    else:
        ground_calib = False
    main(args)