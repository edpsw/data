import os
import argparse
import numpy as np
from isaacgym import gymapi, gymutil, gymtorch
import torch
import time
import pinocchio as pin

class MotionPlayer:
    def __init__(self, args):
        # init args
        self.args = args
        if self.args.robot_type == 'g1':
            urdf_path = "robot_description/g1/g1_29dof_rev_1_0.urdf"
            self.robot = pin.RobotWrapper.BuildFromURDF('robot_description/g1/g1_29dof_rev_1_0.urdf', 'robot_description/g1', pin.JointModelFreeFlyer())
            self.Tpose = np.array([0,0,0.785,0,0,0,1,
                                    -0.15,0,0,0.3,-0.15,0,
                                    -0.15,0,0,0.3,-0.15,0,
                                    0,0,0,
                                    0, 1.57,0,1.57,0,0,0,
                                    0,-1.57,0,1.57,0,0,0]).astype(np.float32)
        elif self.args.robot_type == 'h1_2':
            urdf_path = "robot_description/h1_2/h1_2_wo_hand.urdf"
        elif self.args.robot_type == 'h1':
            urdf_path = "robot_description/h1/h1.urdf"
        
        # inital gym
        self.gym = gymapi.acquire_gym()
        # create sim environment
        self.sim = self._create_simulation()
        # add plane
        self._add_ground_plane()
        # load urdf
        self.asset = self._load_urdf(urdf_path)
        # create and add robot
        self.env = self._create_env_with_robot()

    def _create_simulation(self):
        """create physics simulation environment"""
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 30.0
        sim_params.gravity = gymapi.Vec3(0.0, 0, -9.81)
        sim_params.up_axis = gymapi.UP_AXIS_Z
        
        return self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

    def _add_ground_plane(self):
        """add plane"""
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)  # Z-up plane
        plane_params.distance = 0                   # the distance from plane to original
        plane_params.static_friction = 1.0
        plane_params.dynamic_friction = 1.0
        self.gym.add_ground(self.sim, plane_params)

    def _load_urdf(self, urdf_path):
        """load URDF"""
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not existent: {urdf_path}")
            
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
        
        asset_root = os.path.dirname(urdf_path)
        asset_file = os.path.basename(urdf_path)
        
        return self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

    def _create_env_with_robot(self):
        """create environment with robot"""
        env = self.gym.create_env(self.sim, 
                                 gymapi.Vec3(-2, 0, -2), 
                                 gymapi.Vec3(2, 2, 2), 
                                 1)
        
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.0)  # put on the place with 1 meter high
        self.actor = self.gym.create_actor(env, self.asset, pose, "Robot", 0, 0)
        
        return env

    def set_camera(self, viewer):
        """ set the camera"""
        cam_pos = gymapi.Vec3(3, 2, 3)
        cam_target = gymapi.Vec3(0, 0, 1)
        self.gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)
        
    def run_viewer(self):
        """run visualize"""
        # create viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if not self.viewer:
            return
            
        self.set_camera(self.viewer)
        motion_data = self.load_data()
        
        root_state_tensor = torch.zeros((1, 13), dtype=torch.float32)
        dof_state_tensor = torch.zeros((29, 2), dtype=torch.float32)
        
        # main loop
        while not self.gym.query_viewer_has_closed(self.viewer):
            for frame_nr in range(motion_data.shape[0]):
                start_time = time.time()
                configuration = torch.from_numpy(motion_data[frame_nr, :])
                print("###############", configuration.shape)
                self.get_key_point(motion_data[frame_nr, :])
                self.get_robot_state()
                
                root_state_tensor[0, :7] = configuration[:7]
                dof_state_tensor[:,0] = configuration[7:]
                
                self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(root_state_tensor))
                self.gym.set_dof_state_tensor(self.sim, gymtorch.unwrap_tensor(dof_state_tensor))
                
                self.gym.step_graphics(self.sim)
                self.gym.draw_viewer(self.viewer, self.sim, True)
                elapsed_time = time.time() - start_time
                sleep_time = max(0, 1.0 / 30.0 - elapsed_time)
                time.sleep(sleep_time)
        
        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)

    def load_data(self):
        file_name = self.args.file_name
        robot_type = self.args.robot_type
        csv_files = robot_type + '/' + file_name + '.csv'
        data = np.genfromtxt(csv_files, delimiter=',')
        
        return data
    
    # key point visualization
    def clear_lines(self):
        self.gym.clear_lines(self.viewer)

    def draw_sphere(self, pos, radius, color, env_id, pos_id=None):
        sphere_geom_marker = gymutil.WireframeSphereGeometry(radius, 20, 20, None, color=color)
        sphere_pose = gymapi.Transform(gymapi.Vec3(pos[0], pos[1], pos[2]), r=None)
        gymutil.draw_lines(sphere_geom_marker, self.gym, self.viewer, self.env, sphere_pose)

    def draw_line(self, start_point, end_point, color, env_id):
        gymutil.draw_line(start_point, end_point, color, self.gym, self.viewer, self.env)
    
    # get key point by piconicon
    def get_key_point(self, configuration = None):
        self.clear_lines()
        self.robot.framesForwardKinematics(self.Tpose if configuration is None else configuration)
        
        count = 0
        for visual in self.robot.visual_model.geometryObjects:
            frame_name = visual.name[:-2]
            frame_id = self.robot.model.getFrameId(frame_name)
            parent_joint_id = self.robot.model.frames[frame_id].parentJoint
            parent_joint_name = self.robot.model.names[parent_joint_id]
            joint_tf = self.robot.data.oMi[parent_joint_id]
            
            ref_body_pos = joint_tf.translation 
            ref_body_rot = joint_tf.rotation
            count += 1
            
            color_inner = (0.0, 0.0, 0.545)
            color_inner = tuple(color_inner)
            
            # import ipdb; ipdb.set_trace()
            self.draw_sphere(ref_body_pos, 0.04, color_inner, 0)

    def get_robot_state(self):
        rigid_body_state = self.gym.acquire_rigid_body_state_tensor(self.sim)
        self._rigid_body_state = gymtorch.wrap_tensor(rigid_body_state)

        self.num_bodies = self.gym.get_asset_rigid_body_count(self.asset)
        self.num_envs = 1
        bodies_per_env = self._rigid_body_state.shape[0] // self.num_envs
        self._rigid_body_state_reshaped = self._rigid_body_state.view(self.num_envs, bodies_per_env, 13)
        self._rigid_body_pos = self._rigid_body_state_reshaped[..., :self.num_bodies, 0:3]
        self._rigid_body_rot = self._rigid_body_state_reshaped[..., :self.num_bodies, 3:7]
        self._rigid_body_vel = self._rigid_body_state_reshaped[..., :self.num_bodies, 7:10]
        self._rigid_body_ang_vel = self._rigid_body_state_reshaped[..., :self.num_bodies, 10:13]
        
    def compute_angular_velocity(joint_angles, frame_rate):
        """
        Compute joint angular velocity using central difference.
        
        Args:
            joint_angles: (N, num_joints) array, joint angles in radians.
            frame_rate: Hz, sampling frequency.
        
        Returns:
            angular_velocity: (N, num_joints) array, rad/s.
        """
        dt = 1.0 / frame_rate
        N = joint_angles.shape[0]
        angular_velocity = np.zeros_like(joint_angles)
        
        # Central difference for interior points
        angular_velocity[1:-1] = (joint_angles[2:] - joint_angles[:-2]) / (2 * dt)
        
        # Forward/backward difference for boundaries
        angular_velocity[0] = (joint_angles[1] - joint_angles[0]) / dt
        angular_velocity[-1] = (joint_angles[-1] - joint_angles[-2]) / dt
        
        return angular_velocity


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_name', type=str, help="File name", default='dance1_subject2')
    parser.add_argument('--robot_type', type=str, help="Robot type", default='g1')
    args = parser.parse_args()
    
    loader = MotionPlayer(args)
    loader.run_viewer()