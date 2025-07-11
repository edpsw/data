import numpy as np
import pickle as p
 
# a = np.load("/home/mycode/exp-humanoid/ASE/ase/poselib/data/npy/01_01.npy")
a = np.load("/home/mycode/exp-humanoid/ASE/ase/poselib/data/retarget_npy/g1/05_02.npy", allow_pickle=True)

# f = open('./xsub/val_label.pkl','rb')
# b = p.load(f)
# b = list(b) #将b转换为list类型，才能转换成numpy类型
# b = np.array(b)
print(a.shape)
# print(b.shape)c


