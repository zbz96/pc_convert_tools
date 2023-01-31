"""
    zhaobingzhen@tongxin.cn
    功能：
        地面校正
            操作步骤：
                1. 按“-”改变点云size
                2. 调整至合适视角，按k锁定视角
                3. 按住ctrl不动，鼠标左键选择区域，松开ctrl自动闭合区域，按C得到所选区域，按S保存为ground.ply文件。
                4. 关闭open3d窗口
                
"""
from curses import window
import  os
import sys
import argparse
import numpy as np
import open3d as o3d

def crop_geometry(file):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry")
    print("5) Press 'S' to save the selected geometry")
    print("6) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries_with_editing([pcd])

def pick_ground_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def main():
    file = "concat_pcd/1659679399.601329327.pcd"
    pcd = o3d.io.read_point_cloud(file)
    #pick_ground_points
    crop_geometry(file)
    ground_points = pick_ground_points(pcd)


    #load ground.ply
    ply_path = os.path.join("ground.ply")
    print(ply_path)
    if not os.path.exists(ply_path):
        print("the ground ply does not exist")
        sys.exit(-1)
    #TODO

 






    # plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=5,num_iterations=10000)
    # [a,b,c,d] = plane_model
    # print(a,b,c,d)


if __name__ == '__main__':
    main()