"""
    author:
        zhaobingzhen@tongxin.cn
    功能:
        bagConvert，不依赖ros/pcl环境
            bag2pcd 
            bag2bin 
            bag2img
"""

import os
import argparse
import ros_numpy
import sensor_msgs
import numpy as np
import rosbag
import cv2

def convert_pc_msg_to_np(pc_msg,remove_nans=True):
    pc_msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg, squeeze=True)
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
        # pull out x, y, and z  intensity
        pc_np = np.zeros(cloud_array.shape + (4,), dtype=np.float32)
        pc_np[...,0] = cloud_array['x']
        pc_np[...,1] = cloud_array['y']
        pc_np[...,2] = cloud_array['z']
        pc_np[...,3] = cloud_array['intensity']

    return pc_np

def bag2img(path, img_msg):
    image_data = np.frombuffer(img_msg.data, dtype=np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    imgFileName = path + str(img_msg.header.stamp.secs) +'.'+str(img_msg.header.stamp.nsecs) +'.png'
    cv2.imwrite(imgFileName, image)

def bag2pcd(path, msg, pc_np):

    pcdFileName = path + str(msg.header.stamp.secs) +'.'+str(msg.header.stamp.nsecs) +'.pcd'
    #写文件句柄
    handle = open(pcdFileName, 'a')
    #得到点云点数
    point_num=pc_np.shape[0]
    #pcd头部（重要）
    handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1')
    string = '\nWIDTH ' + str(point_num)
    handle.write(string)
    handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(point_num)
    handle.write(string)
    handle.write('\nDATA ascii')

    # 依次写入点
    for i in range(point_num):
        string = '\n' + str(pc_np[i, 0]) + ' ' + str(pc_np[i, 1]) + ' ' + str(pc_np[i, 2]) + ' '+ str(pc_np[i,3])
        handle.write(string)
    handle.close()


def bag2bin(path, msg, pc_np):
    binFileName = path + str(msg.header.stamp.secs) +'.'+str(msg.header.stamp.nsecs) +'.bin'
    pc_np.tofile(binFileName)

def main():
    parser =  argparse.ArgumentParser(description="Convert .bag to .pcd or .bin")
    parser.add_argument("--bag_path",help='.bag file path.',type=str,default="")
    parser.add_argument("--mode",help='.bag to .pcd or .bag to .bin',type=str,default="bag2pcd")
    parser.add_argument("--pcd_path",help="save pcd path",type=str,default="")
    parser.add_argument("--bin_path",help="save bin path",type=str,default="")
    parser.add_argument("--lidar_topic",help="lidar topic name",type=str,default="")
    parser.add_argument("--camera_topic",help="camera topic name",type=str,default="")
    parser.add_argument("--img_path",help="save img path",type=str,default="")
    args = parser.parse_args()
    
    if len(args.pcd_path) != 0:
        if os.path.exists(args.pcd_path) == False:
            os.mkdir(args.pcd_path)
    if len(args.bin_path) != 0:
        if os.path.exists(args.bin_path) == False:
            os.mkdir(args.bin_path)
    if len(args.img_path) != 0:
        if os.path.exists(args.img_path) == False:
            os.mkdir(args.img_path)
    
    #Load bagfile
    bag_files = []
    for dir in  os.listdir(args.bag_path):
        bag_files.append(args.bag_path + dir)

    for bag_file in bag_files:
        print("Start to convert %s in %s" %(args.lidar_topic, bag_file))
        for topic, msg, t in rosbag.Bag(bag_file).read_messages():
            if topic == args.lidar_topic:
                pc_np= convert_pc_msg_to_np(msg,remove_nans=True)
                if args.mode =='bag2pcd':
                    bag2pcd(args.pcd_path,msg,pc_np)
                if args.mode == 'bag2bin':
                    bag2bin(args.bin_path, msg,pc_np)
            if topic == args.camera_topic:
                bag2img(args.img_path, msg)
        print("Ending convert %s in %s" %(args.lidar_topic, bag_file))
if __name__ == '__main__':
    main()