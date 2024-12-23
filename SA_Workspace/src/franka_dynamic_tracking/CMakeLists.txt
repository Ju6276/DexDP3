cmake_minimum_required(VERSION 3.10)
project(franka_dynamic_tracking)

add_compile_options(-std=c++11 -DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  rospy
  std_msgs
  bio_ik
  moveit_ros_robot_interaction
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Curses REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    roscpp
    rospy
    std_msgs
    bio_ik
    moveit_ros_planning
    moveit_ros_robot_interaction
    moveit_ros_planning_interface
    geometry_msgs
    tf2
    tf2_geometry_msgs 
    tf2_eigen 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)


add_executable(move_init src/move_init.cpp)
target_link_libraries(move_init ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mimic_pose_publisher src/mimic_pose_publisher.cpp)
target_link_libraries(mimic_pose_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(end_effector_tracking src/end_effector_tracking.cpp)
target_link_libraries(end_effector_tracking ${catkin_LIBRARIES})
