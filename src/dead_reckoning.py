import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
import utils

def load_KITTI_txt(path, id_list):
    file_names = glob.glob(path + '/*.txt')
    file_names.sort()
    for i in range(len(file_names)):
        file1 = open(file_names[i], 'r')
        Lines = file1.readlines()
        str1 = Lines[0][:-1]
        str1 = str1.split(' ')
        float1 = np.array([float(j) for j in str1])
        float1 = float1[id_list]
        yield float1

def get_KITTI_size(path):
    file_names = glob.glob(path + '/*.txt')
    return len(file_names)

def save_KITTI_to_np(loader, size, name):
    data = next(loader)
    l = data.shape[0]
    np_array = np.zeros([size, l])
    np_array[0] = data
    for i in range(1, size):
        data = next(loader)
        np_array[i] = data
    np.save(name, np_array)

Data_Loader = load_KITTI_txt('../KITTI/2011_09_30/2011_09_30_drive_0020_sync/oxts/data', [8,9,10,17,18,19])
Data_Loader_lalo = load_KITTI_txt('../KITTI/2011_09_30/2011_09_30_drive_0020_sync/oxts/data', [0,1])
number_of_data = get_KITTI_size('../KITTI/2011_09_30/2011_09_30_drive_0020_sync/oxts/data')

# extract imu data to npy, only need to do once.
#save_KITTI_to_np(Data_Loader, number_of_data, '../KITTI/2011_09_30/Odometry_data/imu')


IMU_hz = 10
pose_init = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
pos = np.zeros([number_of_data, 2])
# plot deadrecon by IMU
for i in range(number_of_data):
    pos[i,:] = np.array([pose_init[0,3], pose_init[1,3]])
    # update SE3
    IMU = next(Data_Loader)
    pose_init = pose_init @ expm(utils.twistHat(IMU)/IMU_hz)

# plot deadrecon by Visual odometry
VO = np.load('../KITTI/2011_09_30/Odometry_data/vo.npy')
VO_hz = 10
pose_init = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
pos_vo = np.zeros([number_of_data, 2])
for i in range(VO.shape[0]):
    pos_vo[i,:] = np.array([pose_init[0,3], pose_init[1,3]])
    # update SE3
    pose_init = pose_init @ expm(utils.twistHat(VO[i])/VO_hz)

# plot GPS
r = 6.3781 * (10**6)
gt_pose = np.zeros([number_of_data, 2])
pre_GPS = next(Data_Loader_lalo)
for i in range(1,number_of_data):
    cur_GPS = next(Data_Loader_lalo)
    if i != 0:
        dx_th = (cur_GPS[1] - pre_GPS[1]) * np.pi/180
        dy_th = (cur_GPS[0] - pre_GPS[0]) * np.pi/180
        gt_pose[i] = gt_pose[i-1] + np.array([dx_th, dy_th]) * r
    pre_GPS = cur_GPS


# plot dead of IMU
plt.scatter(pos[:,0], pos[:,1], s = 1)
# plot dead of VO
plt.scatter(pos_vo[:,0], pos_vo[:,1], s = 1)
# plot GPS
#plt.scatter(gt_pose[:,0], gt_pose[:,1], s = 1)
#plt.axis('equal')
plt.show()
