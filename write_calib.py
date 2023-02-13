"""
Date:2023.02.09
author: zhaobingzhen@tongxin.cn
describe: this script is used for write kitti calib
"""
import os 
import numpy as np

def writeCalib(calib_file):

    P2 = np.array([ 7.215377000000e+02,0.000000000000e+00, 6.095593000000e+02, 0.000000000000e+00,
                                 0.000000000000e+00,7.215377000000e+02, 1.728540000000e+02, 0.000000000000e+00, 
                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00])
    Tr_velo_to_cam =  np.array([0.0,-1.0,0.0,0.0,
                                                                0.0, 0.0, -1.0,0.0,
                                                                1.0, 0.0, .00, 0.0
                                                                ])
    Tr_imu_to_velo =np.array([
                                9.999976000000e-01, 7.553071000000e-04, -2.035826000000e-03, -8.086759000000e-01,
                                -7.854027000000e-04, 9.998898000000e-01, -1.482298000000e-02, 3.195559000000e-01, 
                                2.024406000000e-03, 1.482454000000e-02, 9.998881000000e-01, -7.997231000000e-01])
    with open(calib_file,'w') as f:

        f.writelines("P0: ")
        for num in P2.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")

        f.writelines("P1: ")        
        for num in P2.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")

        f.writelines("P2: ")
        for num in P2.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")

        f.writelines("P3: ")
        for num in P2.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")

        f.writelines("R0_rect: ")
        for num in np.eye(3,3).flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")
        
        f.writelines('Tr_velo_to_cam: ')
        for num in Tr_velo_to_cam.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")

        f.writelines('Tr_imu_to_velo: ')
        for num in Tr_imu_to_velo.flatten():
            f.writelines(str(num)+ " ")
        f.writelines("\n")



        

