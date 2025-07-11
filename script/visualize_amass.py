import numpy as np
import time
import mujoco
import mujoco.viewer


def mj_play(data, fr):
    m = mujoco.MjModel.from_xml_path('/home/z/code/FALCON/humanoidverse/data/robots/g1/g1_29dof.xml')
    m.opt.timestep = 1.0/fr
    d = mujoco.MjData(m)

    len = data.shape[0]
    step = 0
    # # 获取预定义相机ID [1,3](@ref)
    # camera_name = "top_cam"
    # camera_id = mujoco.mj_name2id(
    #     m, 
    #     mujoco.mjtObj.mjOBJ_CAMERA, 
    #     camera_name
    # )

    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
        # if camera_id != -1:
        #     viewer.cam.fixedcamid = camera_id
        #     print(f"✅ 已激活相机: {camera_name} (ID={camera_id})")
        # else:
        #     print(f"⚠️ 相机未找到: {camera_name}, 使用默认视角")
        while viewer.is_running() and step<len:
            
            step_start = time.time()

            d.qpos = data[step, :7+29]
            d.qpos[3] = data[step, 6] # to w first
            d.qpos[4:7] = data[step, 3:6]

            mujoco.mj_forward(m, d)
            viewer.sync()
            # Rudimentary time keeping, will drift relative to wall clock.
            step += 1
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

def read_rtj(fpath):
    #get frame rate from file name
    fr = fpath[-12:-9]
    fr = int(fr[1:]) if fr[0]=='_' else int(fr)

    jpos = np.load(fpath)
    jpos[:,2] += 0.793
    return jpos,fr




# if __name__ == '__main__':
#     jpos, fr = read_rtj('/home/mycode/data/Retargeted_AMASS_for_robotics/g1/KIT/3/bend_left06_poses_100_jpos.npy')
#     mj_play(jpos, fr)
  

import os
import argparse
import numpy as np

def main():
    # 设置参数解析器
    print("python /home/z/code/data/script/visualize2_amass.py --start 1 --step 5")
    parser = argparse.ArgumentParser(description='播放运动序列')
    parser.add_argument('--start', type=int, default=0, help='起始文件序号')
    parser.add_argument('--step', type=int, default=1, help='读取序号步长')
    args = parser.parse_args()
    
    # 根目录路径
    root_dir = '/home/z/code/data/Retargeted_AMASS_for_robotics/g1'
    
    # 收集所有npy文件路径
    file_paths = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.npy'):
                file_paths.append(os.path.join(root, file))
    
    # 按文件名排序
    file_paths.sort()
    
    # 根据用户指定的起始位置和步长筛选文件
    selected_files = file_paths[args.start::args.step]
    
    print(f"找到 {len(file_paths)} 个运动序列文件")
    print(f"从索引 {args.start} 开始，以步长 {args.step} 播放 {len(selected_files)} 个文件")
    
    # 播放选中的文件
    for i, file_path in enumerate(selected_files):
        print(f"\n[{i * args.step + args.start}/{len(file_paths)}] 播放文件: {file_path}")
        try:
            # 读取文件
            jpos, fr = read_rtj(file_path)
            # 播放运动序列
            mj_play(jpos, fr)
            print(f"√ 成功播放 {os.path.basename(file_path)}")
        except Exception as e:
            print(f"× 播放失败: {str(e)}")
            break

if __name__ == '__main__':
    main()


# xingzou zhaoshou
#5505 248K	/home/mycode/data/Retargeted_AMASS_for_robotics/g1/BioMotionLab_NTroje/rub108/0013_knocking1_poses_120_jpos.npy