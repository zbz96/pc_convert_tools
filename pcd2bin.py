"""
    author:
        zhaobingzhen@tongxin.cn
    功能:
        pcd2bin
"""
import numpy as np
import os
import argparse
from pypcd import pypcd


def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument("--pcd_path",help=".pcd file path.",type=str,default="/home/user/lidar_pcd")
    parser.add_argument("--bin_path",help=".bin file path.",type=str,default="/home/user/lidar_bin")
    args = parser.parse_args()

    ## Load pcd files
    pcd_files = []
    for (path, dir, files) in os.walk(args.pcd_path):
        for filename in files:
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)

    print("Start to Convert bin to pcd")

    for pcd_file in pcd_files:
        ## Get pcd file
        pc = pypcd.PointCloud.from_path(pcd_file)

        binFileName = args.pcd_path + (pcd_file.split('/')[-1]).split('.')[0] + '.bin'
        ## Get data from pcd (x, y, z, intensity, ring, time)
        np_x = (np.array(pc.pc_data['x'], dtype=np.float32)).astype(np.float32)
        np_y = (np.array(pc.pc_data['y'], dtype=np.float32)).astype(np.float32)
        np_z = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32)/256
        # np_r = (np.array(pc.pc_data['ring'], dtype=np.float32)).astype(np.float32)
        # np_t = (np.array(pc.pc_data['time'], dtype=np.float32)).astype(np.float32)

        ## Stack all data    
        points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i)))

        ## Save bin file                                    
        points_32.tofile(binFileName)
    
if __name__ == "__main__":
    main()
