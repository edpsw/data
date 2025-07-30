# Step 1: Set up a Conda virtual environment
conda --version
conda config --set allow_conda_downgrades true
conda install conda=4.6.14

conda create -n retarget python=3.10
conda activate retarget

# Step 2: Install dependencies

conda install pinocchio -c conda-forge
pip install numpy rerun-sdk==0.22.0 trimesh

# Step 3: Run the script
python rerun_visualize.py
# run the script with parameters:
# 
python rerun_visualize.py --file_name dance1_subject2 --robot_type g1
#[g1|h1|h1_2]
