import numpy as np
import pickle as p
dir = "/home/mycode/data/AMASS_Complete/DanceDB/20140121_FanieAlexandrou/Fanie_Zumba_C3D_poses.npz"
data = np.load(dir)
arrays = data.items()
# 打印数组名称和形状
print(dir)
for name, array in arrays:
    print(f'Array name: {name}')
    print(f'Array shape: {array.shape}')
    print(f'Array shape: {type(array)}')
