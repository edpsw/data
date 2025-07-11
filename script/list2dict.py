import numpy as np


# temp = np.load(f"./dance1_subject2.npy").astype(np.float64).tolist()
# a = temp[10]

# a = [-0.190791, 0.179874, 0.319062, 0.44664, -0.278292, -0.041495, -0.200741, -0.210048, -0.320937, 0.416376, -0.300216, 0.093906, -0.005507, -0.00318, 0.002137, -0.171327, 1.710336, 0.134352, 1.171921, -0.00957, 0.010702, 0.370233, -0.145459, -1.66146, -0.083894, 1.117313, -0.096474, -0.04809, -0.306177]
a = [0.0] * 29
# default_joint_angles = {
#                 "left_hip_pitch_joint": -0.190791,
#                 "left_hip_roll_joint": 0.179874,
#                 "left_hip_yaw_joint": 0.319062,
#                 "left_knee_joint": 0.44664,
#                 "left_ankle_pitch_joint": -0.278292,
#                 "left_ankle_roll_joint": -0.041495,
#                 "right_hip_pitch_joint": -0.200741,
#                 "right_hip_roll_joint": -0.210048,
#                 "right_hip_yaw_joint": -0.320937,
#                 "right_knee_joint": 0.416376,
#                 "right_ankle_pitch_joint": -0.300216,
#                 "right_ankle_roll_joint": 0.093906,
#                 "waist_yaw_joint": -0.005507,
#                 "waist_roll_joint": -0.00318,
#                 "waist_pitch_joint": 0.002137,
#                 "left_shoulder_pitch_joint": -0.171327,
#                 "left_shoulder_roll_joint": 1.710336,
#                 "left_shoulder_yaw_joint": 0.134352,
#                 "left_elbow_joint": 1.171921,
#                 "left_wrist_roll_joint": -0.00957,
#                 "left_wrist_pitch_joint": 0.010702,
#                 "left_wrist_yaw_joint": 0.370233,
#                 "right_shoulder_pitch_joint": -0.145459,
#                 "right_shoulder_roll_joint": -1.66146,
#                 "right_shoulder_yaw_joint": -0.083894,
#                 "right_elbow_joint": 1.117313,
#                 "right_wrist_roll_joint": -0.096474,
#                 "right_wrist_pitch_joint": -0.04809,
#                 "right_wrist_yaw_joint": -0.306177
#             }


# # 按顺序更新字典值
# keys = list(default_joint_angles.keys())

# for idx, key in enumerate(keys):
#     default_joint_angles[key] = a[idx]

# # 换行打印
# import json
# print(json.dumps(default_joint_angles, indent=4))



dof_names = {
    "0": "left_hip_pitch_joint",
    "1": "left_hip_roll_joint",
    "2": "left_hip_yaw_joint",
    "3": "left_knee_joint",
    "4": "left_ankle_pitch_joint",
    "5": "left_ankle_roll_joint",
    "6": "right_hip_pitch_joint",
    "7": "right_hip_roll_joint",
    "8": "right_hip_yaw_joint",
    "9": "right_knee_joint",
    "10": "right_ankle_pitch_joint",
    "11": "right_ankle_roll_joint",
    "12": "waist_yaw_joint",
    "13": "left_shoulder_pitch_joint",
    "14": "left_shoulder_roll_joint",
    "15": "left_shoulder_yaw_joint",
    "16": "left_elbow_joint",
    "17": "left_wrist_roll_joint",
    "18": "right_shoulder_pitch_joint",
    "19": "right_shoulder_roll_joint",
    "20": "right_shoulder_yaw_joint",
    "21": "right_elbow_joint",
    "22": "right_wrist_roll_joint"
}

# 按顺序更新字典值
default_joint_angles = {}
keys = list(dof_names.values())
for idx, key in enumerate(keys):
    default_joint_angles[key] = a[idx]

# 换行打印
import json
print(json.dumps(default_joint_angles, indent=4))