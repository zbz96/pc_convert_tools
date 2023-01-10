"""
    author:
        zhaobingzhen@tongxin.cn
    功能:
        bin2pcd
"""
import numpy as np
import os
import argparse


def bin2pcd(binFile,pcdFileName):
    points = np.fromfile(str(binFile), dtype=np.float32).reshape(-1, 4)
    handle = open(pcdFileName, 'a')
    #得到点云点数
    point_num=points.shape[0]
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
        string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2]) + ' '+ str(points[i,3])
        handle.write(string)
    handle.close()

def main():
    parser = argparse.ArgumentParser(description="Convert .bin to .pcd")
    parser.add_argument("--bin_path",help='.bin file path.',type=str,default="/home/user/lidar_bin")
    parser.add_argument("--pcd_path",help='.pcd file path.',type=str,default="/home/user/lidar_pcd")
    args = parser.parse_args()
    
    #Load  bin_files
    bin_files = []
    for (path, dir, files) in os.walk(args.bin_path):
        for filename in files:
            ext = os.path.splitext(filename)[-1]
            if ext == '.bin':
                bin_files.append(path+ filename)

    if not os.path.exists(args.bin_path):
        print("Please check if the bin file exists")
    if not os.path.exists(args.pcd_path):
        os.mkdir(args.pcd_path)
    
    print("Start to Convert bin to pcd")
    #convert
    for bin_file in bin_files:
        suffix = bin_file.split('/')[-1].split('.')[-1] #bin
        pcdFileName = args.pcd_path + bin_file.split('/')[-1].replace(suffix,'pcd')
        bin2pcd(bin_file,pcdFileName)

if __name__ == '__main__':
    main()