"""
    author：
        zhaobingzhen@tongxin.cn
    功能：  
        点云拼接
"""
import os
import argparse
import open3d as o3d
import numpy as np


left_trans = np.array([[0.681137, -0.731433, -0.0325158, 0.162936], 
                        [0.732083, 0.681028, 0.0160683, 1.2285], 
                        [0.0103913, -0.0347491, 0.999343, -1.69266], 
                        [0, 0, 0, 1]])

right_trans = np.array([[0.675851, 0.736569, -0.0263177, 0.149243], 
                        [-0.736983, 0.675811, -0.0117303, -1.2687], 
                        [0.00914559, 0.0273236, 0.999585, -1.68882], 
                        [0, 0, 0, 1]])


def savePCD(pc_np,file):
    handle = open(file,'a')
    handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1')
    point_num = pc_np.shape[0]
    string = '\nWIDTH ' + str(point_num)
    handle.write(string)
    handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(point_num)
    handle.write(string)
    handle.write('\nDATA ascii')
    for i in range(point_num):
        string = '\n' + str(pc_np[i, 0]) + ' ' + str(pc_np[i, 1]) + ' ' + str(pc_np[i, 2]) + ' '+ str(1.0)
        handle.write(string)
    handle.close()

def read_points(pcd_file):
    pcd = o3d.io.read_point_cloud(pcd_file)
    assert len(pcd.points) > 0
    return np.asarray(pcd.points)

def trans(points,rotMat):
    rot_points = np.dot(points, rotMat[:3, :3].T) + rotMat[:3, 3]
    return rot_points

def pc_concat(path,top_file,left_file,right_file):
    top_pc = read_points(top_file)
    left_pc = read_points(left_file)
    right_pc = read_points(right_file)
    
    left_pc = trans(left_pc,left_trans)
    right_pc = trans(right_pc,right_trans)
    pc_concat = np.concatenate((top_pc, right_pc), axis=0).astype(np.float32)
    pcdFileName = path + top_file.split('/')[-1]
    savePCD(pc_concat,pcdFileName)

def main():
    parser = argparse.ArgumentParser(description="PointCloudConcatenate")
    parser.add_argument("--top_pcd_path",help="The top lidar pcd file path",type=str,default="")
    parser.add_argument("--right_pcd_path",help="The right lidar pcd file path",type=str,default="")
    parser.add_argument("--left_pcd_path",help="The left lidar pcd file path",type=str,default="")
    parser.add_argument("--concat_pcd_path",help="output concat_pc pcd file path",type=str,default="concat_pcd/")
    args = parser.parse_args()

    if not os.path.exists(args.top_pcd_path) or not os.path.exists(args.right_pcd_path) or not os.path.exists(args.left_pcd_path):
        print("Please check if the PCD file exists")
    
    if not os.path.exists(args.concat_pcd_path):
        os.mkdir(args.concat_pcd_path)
    
    top_pcd_files = []
    left_pcd_files = []
    right_pcd_files = []

    for dir in os.listdir(args.top_pcd_path):
        top_pcd_files.append(args.top_pcd_path + dir)
    top_pcd_files.sort()
    print("The number of top_pcd_file is %s" %len(top_pcd_files))
    for dir in os.listdir(args.left_pcd_path):
        left_pcd_files.append(args.left_pcd_path + dir)
    left_pcd_files.sort()
    print("The number of left_pcd_file is %s" %len(left_pcd_files))
    for dir in os.listdir(args.right_pcd_path):
        right_pcd_files.append(args.right_pcd_path + dir)
    right_pcd_files.sort()
    print("The number of right_pcd_file is %s" %len(right_pcd_files))

    assert len(top_pcd_files) == len(left_pcd_files) == len(right_pcd_files), 'The number of PCD is inconsistent'
    
    print("Concatenating the pointcloud")
    for i, top_file in enumerate(top_pcd_files):
        left_file = left_pcd_files[i]
        right_file = right_pcd_files[i]
        pc_concat(args.concat_pcd_path,top_file,left_file,right_file)

if __name__ == '__main__':
    main()