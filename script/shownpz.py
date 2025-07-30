import numpy as np
import pickle as p
# dir = "/home/z/code/PBHC/smpl_retarget/motion_data/lafan/dance2_subject1.npz"
# dir = '/home/z/code/data/amass_npz/ACCAD/Female1General_c3d/A1 - Stand_poses.npz'
dir = '/home/z/code/data/amass_npz/CMU/01/01_02_poses.npz'
data = np.load(dir)
arrays = data.items()
# 打印数组名称和形状
print(dir)
for name, array in arrays:
    print(f'Array name: {name}',end = "||")
    print(f'Array shape: {array.shape}',end = "||")
    print(f'Array shape: {type(array)}')
