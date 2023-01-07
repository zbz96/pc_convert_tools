
import os
import argparse
import ros_numpy
import sensor_msgs
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import rosbag

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

def bag2pcd(path, msg, pc_np):

    pcdFileName = path + str(msg.header.stamp)+'.pcd'
    #写文件句柄
    handle = open(pcdFileName, 'a')
    #得到点云点数
    point_num=pc_np.shape[0]
    #pcd头部（重要）
    handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1')
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
    binFileName = path + str(msg.header.stamp)+'.bin'
    pc_np.tofile(binFileName)

def main():
    parser =  argparse.ArgumentParser(description="Convert .bag to .pcd or .bin")
    parser.add_argument(
        "--bag_path",
        help='.bag file path.',
        type=str,
        default="/home/user/bag_path"
    )
    parser.add_argument(
        "--mode",
        help='.bag to .pcd or .bag to .bin',
        type=str,
        default="bag2pcd"
    )
    parser.add_argument(
        "--pcd_path",
        help="save pcd path",
        type=str,
        default="/home/user/pcd_path"
    )
    parser.add_argument(
        "--bin_path",
        help="save bin path",
        type=str,
        default="/home/user/bin_path"
    )
    parser.add_argument(
        "--topic",
        help="topic name",
        type=str,
        default="/pc/lidar/top/pointcloud"
    )
    args = parser.parse_args()
    
    #Load bagfile
    bag_files = []
    for dir in  os.listdir(args.bag_path):
        print(dir)
        bag_files.append(args.bag_path + dir)
    print(bag_files)
    print("Start to Convert")
    

    for bag_file in bag_files:
        for topic, msg, t in rosbag.Bag(bag_file).read_messages():
            if topic == args.topic:
                pc_np= convert_pc_msg_to_np(msg,remove_nans=True)
                if args.mode =='bag2pcd':
                    bag2pcd(args.pcd_path,msg,pc_np)
                if args.mode == 'bag2bin':
                    bag2bin(args.bin_path, msg,pc_np)

if __name__ == '__main__':
    main()