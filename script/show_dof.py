from isaacgym import gymapi
from legged_gym import LEGGED_GYM_ROOT_DIR

gym = gymapi.acquire_gym()
sim
asset_root = '/home/mycode/FALCON'
asset_file = 'humanoidverse/data/robots/myh1/urdf/myh1.urdf'
# robot_asset =  gym.load_asset(asset_file)
robot_asset =  gym.load_asset( sim, asset_root, asset_file, asset_options)
num_dof =  gym.get_asset_dof_count(robot_asset)
num_bodies =  gym.get_asset_rigid_body_count(robot_asset)
print("num_dof", num_dof)
print(len(num_dof))
print("num_bodies",num_bodies)
print(len(num_bodies))

