# DexDP3
This repository is for the thesis **Dexterous Manipulation on Multi-Fingered Hands Based on 3D Diffusion Policy**.

[**3D Diffusion Policy (DP3)**](https://github.com/YanjieZe/3D-Diffusion-Policy/tree/master) is a novel generative framework for robotic behavior synthesis utilizing 3D point clouds as input. The framework is implemented on the Franka Panda robotic arm and the Allegro dexterous hand. 

## Video Demonstration
📹 Watch the project in action: [YouTube Link](https://youtu.be/FM8L9Usz-KI?si=CQtNt1eZG-jtY8vq)
## Features  

- **Teleoperation System for Dataset Collection**:  
  - Wrist tracking using the OptiTrack V120:Trio.  
  - Finger motion capture with Manus Quantum Mocap Metagloves.  
  - Data collection with using Realsense LiDAR L515 camera.

- **Unified ROS Workspace**:  
  - Seamless integration of robotic arm and dexterous hand control.  
  - Teleoperation workflows and policy deployment.  

- **Simulation and Real-World Validation**:  
  - Realistic models and environments constructed with MoveIt for safe data collection and deployment.  
  - Policy performance validated on the Franka Panda robotic arm and Allegro dexterous hand.  


---

## System Requirements
- **Robot System Execution**: Ubuntu 22.04
- **Network Training**: NVIDIA 4080 GPU

## Usage Instructions

The project is divided into three main parts: **Teleoperation**, **DP3 Training**, and **Deployment**.

### 1. Teleoperation

#### 1.1 Launch Franka and Allegro Robots with ROS Nodes
Run the following command to launch the ROS nodes:
```bash
roslaunch SA_Workspace/src/frankaAllegro_moveit_config/launch/real_robot.launch
```

#### 1.2 Run the Tracking System
Two versions of the tracking system are available:
- **C++ Version** and **Python Version** can be found in the following path:
  ```bash
  SA_Workspace/src/franka_dynamic_tracking/src
  ```

#### 1.3 Process and Save Data
Navigate to the `data_collection` folder and run the script to process and save the data:
```bash
python real_data_collection.py
```

#### 1.4 Generate Point Clouds
Run the following script to generate point clouds from `img` and `depth` data:
```bash
python generatePointCloud.py
```

#### 1.5 Merge Episodes
Run the script to merge episodes and convert the data into the format required for network input:
```bash
python merge_episodes.py
```
#### 1.6 Estimate the Extrinsic Parameters
This repository provides a script to estimate the extrinsic parameters of the ArUco markers using pyrealsense2. 
Run the script to estimate the extrinsic parameters of the ArUco markers:
```bash
python aruco_extrinsic.py
```
---

### 2. DP3 Training

#### 2.1 Clone the Original DP3 Repository
On the training computer, clone the DP3 repository:
```bash
git clone https://github.com/YanjieZe/3D-Diffusion-Policy.git
```
  
#### 2.2 Download the Dataset
Download the dataset from the following link:
[Download Dataset](https://drive.google.com/drive/folders/16e6ZLCcEMtb3HNePVMUcsi8VkaiUYYDP?usp=sharing)

#### 2.3 Update Repository Files
Place the files from this repository into the corresponding directories of the original DP3 codebase.

#### 2.4 Use the New Visualization Tools
The `visualizer` folder contains updated tools for visualizing point clouds and data formats.

#### 2.5 Start Training
Run the following command to start training:
```bash
bash scripts/train_policy.sh simple_dp3 realdex_push 2248 0 0
```

---

### 3. Deployment

#### 3.1 Run Deployment Script on the Training Computer
On the training computer, run the following command:
```bash
bash scripts/deploy_policy.sh simple_dp3 realdex_push 2248 0 0
```

#### 3.2 Run Control Script on the Execution Computer
On the execution computer, run the following script to control the robot:
```bash
python SA_Workspace/src/control.py
```
#### 3.3 Check the Network Status
Here provide a script to check the communication between the training computer and the execution computer with ROS:
```bash
python SA_Workspace/src/test_ros_network.py
```
---



## Results  

The **3D Diffusion Policy** demonstrates effective performance in advanced robotic manipulation tasks.  


## Contact
For questions or further assistance, please contact Ju Dong at [ju.dong6276@gmail.com].

