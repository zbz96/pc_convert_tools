"""
Date:2023.01.26
author: zhaobingzhen@tongxin.cn
describe: this script is used for pcd2bin
"""
import numpy as np
import os
import argparse

def readPCD(file):
    lines = []
    num_points = None
    with open(file,'r') as f:
        for line in f:
            lines.append(line.strip())
            if line.startswith('POINTS'):
                num_points = int(line.split()[-1])
    assert num_points is not None
    
    points=[]
    for line in lines[-num_points:]:
        x,y,z,intensity = list(map(float,line.split()))
        points.append(np.array([x,y,z,intensity],dtype=np.float32))
    return np.array(points,dtype=np.float32)

def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument("--pcd_path",help=".pcd file path.",type=str,default="/home/user/lidar_pcd")
    parser.add_argument("--bin_path",help=".bin file path.",type=str,default="/home/user/lidar_bin")
    args = parser.parse_args()

    if not os.path.exists(args.pcd_path):
        print("Please check if the PCD file exists")
    if not os.path.exists(args.bin_path):
        os.mkdir(args.bin_path)

    ## Load pcd files
    pcd_files = []
    for (path, dir, files) in os.walk(args.pcd_path):
        for filename in files:
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path+ filename)

    print("Start to Convert bin to pcd")

    for pcd_file in pcd_files:
        points_32 = readPCD(pcd_file)
        suffix = pcd_file.split('/')[-1].split('.')[-1] #pcd
        binFileName = args.bin_path + pcd_file.split('/')[-1].replace(suffix,'bin')
        ## Save bin file                                    
        points_32.tofile(binFileName)
    
if __name__ == "__main__":
    main()
