"""
Date:2023.02.03
author: zhaobingzhen@tongxin.cn
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
import random
import numpy as np
import open3d as o3d


def read_pcd(path):
    pcd = o3d.io.read_point_cloud(path)
    pcd.remove_non_finite_points()
    return np.asarray(pcd.points)

def crop_geometry(file):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twnum_inlierse to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left clnum_inliersk for polygon selection")
    print("4) Press 'C' to get a selected geometry")
    print("5) Press 'S' to save the selected geometry")
    print("6) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries_with_editing([pcd])

def augment(xyzs):
    axyz = np.ones((len(xyzs), 4))
    xyzs_len = len(xyzs)
    for ii in range(xyzs_len):
      axyz[ii,:3] = xyzs[ii][:3]
    return axyz

def estimate(xyzs):
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    dist = np.abs(coeffs.dot(augment([xyz]).T)) / ((coeffs[0]**2 + coeffs[1]**2 + coeffs[2] ** 2) ** 0.5)
    return dist < threshold

def run_ransac(data, estimate, is_inlier, sample_size, goal_inliers, max_iterations, random_seed=None):
    max_inliers = 0
    best_model = None
    inlier_points = []
    random.seed(random_seed)
    data = list(data)
    for i in range(max_iterations):
        s = random.sample(data, int(sample_size))
        coeff = estimate(s)
        num_inliers = 0
        for j in range(len(data)):
            if is_inlier(coeff, data[j]):
                num_inliers += 1
                inlier_points.append(data[j])
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_model = coeff
            if num_inliers > goal_inliers:
                break
    print('took iterations:', i+1, 'best model:', best_model, 'explains:', max_inliers)
    return best_model, max_inliers, np.array(inlier_points)
            
def  plane_fit(data):
    thresh = 0.2
    max_iterations = 100
    sample = 3
    goal_inliers = 0.8 * len(data)
    m, b, ground_pc = run_ransac(data, estimate, lambda x, y: is_inlier(x, y, 0.2), sample, goal_inliers, max_iterations)
    return m, b, ground_pc

def compute_rot(A,B,C):
    target_normal = np.array([0, 0, 1])
    nor = np.array([A,B,C])
    mod = np.linalg.norm(nor)
    nor = nor / mod
    angle = np.arccos(np.dot(nor, target_normal))
    print("compute_rot angle: ", angle, angle* 180/ np.pi)

    p_rot = np.cross(nor,target_normal)
    p_rot = p_rot / np.linalg.norm(p_rot)

    sin_ang = np.sin(angle)
    cos_ang = np.cos(angle)
    # rotMat = np.zeros([4,4])
    rotMat = np.zeros([3,3])
    rotMat[0,0] = cos_ang + p_rot[0] * p_rot[0] * (1 - cos_ang)
    rotMat[0,1] = p_rot[0] * p_rot[1] * ( 1 - cos_ang - p_rot[2] * sin_ang)
    rotMat[0,2] = p_rot[1] * sin_ang + p_rot[0] * p_rot[2] * (1-cos_ang)

    rotMat[1,0] = p_rot[2] * sin_ang + p_rot[0] * p_rot[1] * (1 - cos_ang)
    rotMat[1,1] = cos_ang + p_rot[1] * p_rot[1] * (1 - cos_ang)
    rotMat[1,2] = -p_rot[0] * sin_ang + p_rot[1] * p_rot[2] * ( 1 - cos_ang)

    rotMat[2,0] = -p_rot[1] * sin_ang + p_rot[0] * p_rot[2] * (1 - cos_ang)
    rotMat[2,1] = p_rot[0] * sin_ang + p_rot[1] * p_rot[2] * ( 1 - cos_ang)
    rotMat[2,2] = cos_ang + p_rot[2] * p_rot[2] * (1 - cos_ang)
    return angle,rotMat

def genTransformMat(coeff,points,):
    #平面法向量
    A = coeff[0]
    B = coeff[1]
    C = coeff[2]
    #根据法向量求旋转矩阵
    angle,rotMat = compute_rot(A,B,C)
    #求平移矩阵
    rot_pc = np.transpose(np.dot(rotMat, np.transpose(points)))
    hist, tmp = np.histogram(rot_pc[:, 2], bins=150)
    z_avg = tmp[np.argmax(hist) + 1]
    print("gen_rotMat z_avg: ", z_avg)
    origin_z_height = 0
    delta = origin_z_height - z_avg
    transMat = np.zeros([3,1])
    transMat[2] = delta
    finalMat = np.zeros([4,4])
    finalMat[:3,:3] = rotMat
    finalMat[2,3] = delta
    finalMat[3,3] = 1 #地面矫正矩阵并把坐标原点拉回地面
    print("gen_rotMat finalMat: ", finalMat)
    return finalMat

def vis_np_pnts(np_pnts, window_name, Rotmat=None):
    merge_geos = []
    if Rotmat is not None:
        assert len(np_pnts[0]) == 3
        points = np.dot(np_pnts, Rotmat[:3, :3].T) + Rotmat[:3, 3]
        radius = 80
        create_box = o3d.geometry.TriangleMesh.create_box(width= 2 * radius, height= 2 * radius, depth=0.05)
        create_box.paint_uniform_color([1, 1, 151/255])
        create_box.translate(np.array([-radius, -radius, 0]))
        merge_geos += [create_box]
    else:
        points = np_pnts
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    merge_geos += [pcd]
    o3d.visualization.draw_geometries(merge_geos, window_name=window_name)
    return pcd

def ground_calib_func(infile,outfile):
    #load ground.ply
    ply_path = os.path.join("ground.ply")
    print(ply_path)
    if not os.path.exists(ply_path):
        print("the ground ply does not exist,select ground pointcloud from concat_pcd")
        crop_geometry(infile)
        # sys.exit(-1)
    origin_z_height = 0
    ground_pcd = o3d.io.read_point_cloud(ply_path)
    ground_pcd.remove_non_finite_points()
    ground_pc = np.array(ground_pcd.points)
    print(ground_pc.shape)
    #plane fit
    coeff, num_inliers, seg_pc = plane_fit(ground_pc)
    # genRotMatrix
    finalMat = genTransformMat(coeff,seg_pc)
    ori_pc = read_pcd(infile)
    vis_np_pnts(ori_pc, Rotmat=finalMat, window_name="%d m plane check" % origin_z_height)
    # write_txt(outfile,finalMat)
    np.savetxt(outfile,finalMat,fmt = '%.6f')


if __name__ == '__main__':

    pcd_file = os.path.join('concat_pcd/', os.listdir('concat_pcd/')[0])
    mat_file = "ground_calib_mat.txt"
    ground_calib_func(pcd_file,mat_file)