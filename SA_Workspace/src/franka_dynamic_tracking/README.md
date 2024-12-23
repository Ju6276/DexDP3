# Cartesian online controller for dynamic trakcing with UR5 robot setup

## Features
- Control our UR5 robot via 6D pose commands
- Automatically finds and moves to a reasonable starting pose
- Velocity and acceleration limits
- Smooth online control
- Visualization

## Installation
- For now, just use the existing workspace on tams223

## Usage
- Start the robot setup with the new driver `roslaunch tams_ur5_setup_bringup tams_ur5_setup_ur_robot_driver.launch`
- Start the gripper shell `rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py`
- Activate the gripper
- Open the gripper to remove the pushing tool
- Start the cartesian controller by calling `roslaunch dynamic_pushing dynamic_pushing.launch`
- Put the pushing tool pack into the gripper and close it (e.g. by typing `255` and pressing return)
- You can now send cartesian velocity commands to the `/dynamic_pushing/velocity` topic (you can use the python scripts in this repository as examples)
- Parameters can be changed by editing the `config/parameters.yaml` file and restarting the launch file
