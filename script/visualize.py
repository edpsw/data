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

    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
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

if __name__ == '__main__':
    jpos, fr = read_rtj('/home/z/code/data/Retargeted_AMASS_for_robotics/g1/Transitions_mocap/mazen_c3d/dance_push_poses_120_jpos.npy')
    mj_play(jpos, fr)
