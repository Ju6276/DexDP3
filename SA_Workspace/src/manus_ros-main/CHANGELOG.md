## Upcoming release
## 0.5 - 2024-10-24
 * UPDATE MANUS v2.4 SDK
## 0.4 - 2024-10-24
 * MOVE move the teleoperation code to `p2_teleop` package, leave this package only for publishing manus data
 * FIX fix the hand jump by adding a initial hand movement
 * clang-format change to the same as `p2_ros2` package
## 0.3 - 2024-07-02
 * ADD update the code base using ROS2
 * ADD control of the real robot is now done in the ros control framework
 * ADD demo video
 * UPDATE currently we use `follow_joint_trajectoy_controller` for teleoperation, there is some unstable issues here: https://github.com/ros-controls/ros2_controllers/issues/168, in next version, we plan to implement our own controller.

## 0.2 - 2024-06-26
 * Note of development stop for ROS1 in Readme
 * CMakeLists update for better finding the library
## 0.1 - 2024-06-14
 * ADD `manus_ros_driver`, a ROS1 driver to publish human hand motion data
 * ADD `p2_hand_teleop`, a ROS1 teleop node that use Rviz to visualiztion and debug and P2 C-API for hand control
