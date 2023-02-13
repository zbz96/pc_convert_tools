"""
Date:2023.02.09
author: zhaobingzhen@tongxin.cn
describe: this script is used for match img and file files
"""
import os
import numpy as np
import shutil

def createDict(files_folder):
    res = {}
    files = os.listdir(files_folder)
    files.sort()
    for i, file in enumerate(files):
        timestamp_only_s = file.split(".")[0]
        if timestamp_only_s in res:
            res[timestamp_only_s].append(file)
        else:
            if i != 0:
                res[timestamp_only_s] = [files[i - 1]]      # the last frame of the last second
                previous_second = files[i - 1].split(".")[0]
                res[previous_second] += [file]              # add the first frame of this second to the last second
            else:
                res[timestamp_only_s] = []
            res[timestamp_only_s] += [file]                 # add the current frame to the current second
    return res

def name2time(name, timestamp_format="s.x", has_extenstion=True):
    if timestamp_format == "s.x":
        factor = 1
    elif timestamp_format == "ms.x":
        factor = 1000
    ref_time = factor * float(name[:-4]) if has_extenstion else factor * float(name)     # time of the sample, in s 
    return ref_time    

def match(img_path,lidar_path,dst_path,tolerate_offset):
    #img files
    img_dict = createDict(img_path)
    lidar_files = os.listdir(lidar_path)
    lidar_files.sort()
    if not os.path.exists(dst_path):
        os.mkdir(dst_path)
    #match  为每一个file文件找到相对应的img
    for lidar_file in lidar_files:
        lidar_time = name2time(lidar_file,has_extenstion=True)
        lidar_sec = lidar_file.split('.')[0]
        if lidar_sec not in img_dict:
            print("The lidar frame is not match for image %s" %lidar_file)
        else:
            #如果图像中存在该帧雷达时间对应的图片，找到时间最近的图片
            match_id, min_delta_t = -1, np.Inf
            for idx, file in enumerate(img_dict[lidar_sec]):
                img_time = name2time(file,has_extenstion=True)
                delta = abs(lidar_time - img_time) * 1000
                if delta < min_delta_t:
                    match_id = idx
                    min_delta_t = delta #找到该帧点云时间最近的图片
                match_img = img_dict[lidar_sec][match_id]
                srcimg = os.path.join(img_path,match_img)
                desimg = os.path.join(dst_path,lidar_file[:-4] + '.png') #改为top雷达时间
                shutil.copy(srcimg, desimg)
                
if __name__ == '__main__':
    img_path = 'camera0_img/'
    lidar_path = 'concat_pcd/'
    dst_path = 'camera/'
    match(img_path,lidar_path,dst_path,tolerate_offset = 50)