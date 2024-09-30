## System setup on Nvidia Orin without docker 

0. Log in to your Nvidia Orin.
1. Install ROS2 Humble
```
bash script/software_setup/install_ros_humble.sh
```

2. Install sensor SDK and ros2 driver
*  zed camera

SDK download [link](https://download.stereolabs.com/zedsdk/4.1/l4t36.3/jetsons)
```shell
sudo chmod +x ZED_SDK_Tegra_L4T36.3_v4.1.4.zstd.run
./ZED_SDK_Tegra_L4T36.3_v4.1.4.zstd.run silent runtime_only skip_drivers

# ${zed_ws} is the directory you create for zed ros2 pakcage
cd ${zed_ws}/src
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) --packages-up-to zed-ros2-wrapper
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
```

*  rslidar
```shell
# ${zed_ws} is the directory you create for rslidar ros2 pakcage
cd {rslidar_ws}/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev
cd ..
colcon build --packages-up-to rslidar_sdk
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
```

*  hipnuc imu
```shell
# ${imu_ws} is the directory you create for rslidar ros2 pakcage
cd {imu_ws}/src
git clone https://github.com/hipnuc/products.git
cd products/examples/ROS2/hipnuc_ws
colcon build 
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
```

3. Test your sensor
*  zed camera
```shell
# Replace <camera_model> with the model of the camera that you are using: 'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual'.
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

*  rslidar

Set up your lidar first, including ip address, config file in rslidar_sdk (need rebuild). Please refer to the instruction in [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk).
```shell
ros2 launch rslidar_sdk start.py
```

*  hipnuc imu

Set up your imu first. Please refer to the [instruction](https://github.com/hipnuc/products/tree/master/examples/ROS2).
```shell
ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

4. Install ROS2 packages
* airship_chat
```shell
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
pip install pyaudio openai pocketsphinx
```
  
* airship_grasping

* airship_localization
```shell
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

* airship_navigation
```shell
sudo apt-get install ros-humble-nav2-simple-commander
sudo apt-get install ros-humble-navigation2
sudo apt-get install ros-humble-pointcloud-to-laserscan
```

* airship_perception
```shell
sudo apt install libopenblas-dev
```
* airship_planner

* airship_semantic_map

5. Download AIRSHIP package from GitLab
```shell
# ${DIR_AIRSHIP} is the directory you create for AIRSHIP pakcage
mkdir -p ${DIR_AIRSHIP}/src
cd ${DIR_AIRSHIP}/src
git clone https://gitlab.airs.org.cn/embodied-ai/airship.git
```

6. Install python dependencies
* Setup python environment
```shell
conda create -n airship python==3.10
conda activate airship
```

* Navigation software setup
```shell
# Install local planner
## airship_navigation utilizes neo_local_planner2 as the local planner. We retrieve the code from Neobotix's GitHub repository and compile it from the source.
cd ${DIR_AIRSHIP}/src
git clone https://github.com/neobotix/neo_local_planner2.git -b humble
```

* Grasping software setup
```shell
# 1. Install dependencies of airship_perception on Nvidia ORIN 
## Install  PyTorch https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

pip install torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip install torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl
pip install torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

## Set the environment variables as follows
export AM_I_DOCKER=False
export BUILD_WITH_CUDA=True
export CUDA_HOME=/usr/local/cuda-12.2

## Install GoundingDINO, Segment Anything and diffusers
pip install --no-build-isolation -e GroundingDINO
pip install -e segment_anything
pip install --upgrade diffusers[torch]

## Install other python packages
pip install -r ${DIR_AIRSHIP}/src/airship/airship_perception/seg_pkg/requirements.txt
pip install --upgrade transformers

## Download the pretrained weights
where to put weight?
cd ${DIR_AIRSHIP}/src/airship/airship_perception/
mkdir models
git clone https://huggingface.co/google-bert/bert-base-uncased
wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth


# 2. Install dependencies of airship_grasp
## Install dependencies of grasping net
conda install -c conda-forge cvxopt
pip install -r ${DIR_AIRSHIP}/src/airship_grasp/airship_grasp/Scale_Balanced_Grasp/requirements.txt

cd ${DIR_AIRSHIP}/src/airship_grasp/airship_grasp/Scale_Balanced_Grasp/pointnet2
pip install .

cd ../knn
pip install .

cd ../graspnetAPI
pip install .

## Install robot arm and gripper api
cd ${DIR_AIRSHIP}/src/airship_grasp/airship_grasp/
git clone https://github.com/elephantrobotics/pymycobot.git
cd pymycobot
pip install .
```
