#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>


//This version is cpp version of end_effector_tracking

geometry_msgs::PoseStamped wrist_pose;
std::mutex pose_mutex;

void wristPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    wrist_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "end_effector_tracking");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create MoveIt interface
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    // Subscribe to wrist position topic
    ros::Subscriber wrist_pose_sub = node_handle.subscribe("/natnet_ros/wirst_pose/pose", 1, wristPoseCallback);

    // Publish arrival message
    ros::Publisher arrival_pub = node_handle.advertise<std_msgs::String>("end_effector_arrival", 10);

    // Publish trajectory visualization marker
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("trajectory_marker", 10);

    // Initialize trajectory marker
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.ns = "trajectory";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.01;  // line width
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    // Initialize tf2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(50); // 10 Hz loop/30?/50?
    while (ros::ok()) {
        geometry_msgs::PoseStamped current_wrist_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            current_wrist_pose = wrist_pose;
        }

        // Check if a valid wrist position is received
        if (current_wrist_pose.header.stamp.isZero()) {
            ROS_WARN("No valid wrist position received yet.");
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // Get transform
        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", current_wrist_pose.header.frame_id, ros::Time(0));
            tf2::doTransform(current_wrist_pose, current_wrist_pose, transformStamped);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // y offset
        double y_offset = 0.5; //0.5 meter
        current_wrist_pose.pose.position.y -= y_offset;

        // Rotate quaternion: rotate 90 degrees clockwise around z axis
        tf2::Quaternion rotation;
        rotation.setRPY(0, 0, M_PI / 2); // 90 degrees

        // Current orientation quaternion
        tf2::Quaternion current_orientation;
        tf2::fromMsg(current_wrist_pose.pose.orientation, current_orientation);

        // Apply rotation
        tf2::Quaternion new_orientation = rotation * current_orientation;
        new_orientation.normalize();

        // Update orientation
        current_wrist_pose.pose.orientation = tf2::toMsg(new_orientation);
        // Record initial time
        auto start_plan_time = std::chrono::steady_clock::now();
        // Set end-effector target position
        move_group.setPoseTarget(current_wrist_pose, "franka_flange");

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // End time
        auto end_plan_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> plan_duration = end_plan_time - start_plan_time;


        if (success) {
            auto start_exec_time = std::chrono::steady_clock::now();
            
            move_group.execute(plan);

            auto end_exec_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> exec_duration = end_exec_time - start_exec_time;

            ROS_INFO_STREAM("Planning time: " << plan_duration.count() << " seconds");
            ROS_INFO_STREAM("Execution time: " << exec_duration.count() << " seconds");


            // Print target joint command
            const std::vector<double>& joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
            ROS_INFO_STREAM("Joint target positions: ");
            for (size_t i = 0; i < joint_values.size(); ++i) {
                ROS_INFO_STREAM("Joint " << i << ": " << joint_values[i]);
            }
            // Record trajectory
            ROS_INFO_STREAM("End-effector moved to: " << current_wrist_pose.pose.position.x << ", "
                                                      << current_wrist_pose.pose.position.y << ", "
                                                      << current_wrist_pose.pose.position.z);
            ROS_INFO_STREAM("Quaternion: " << current_wrist_pose.pose.orientation.x << ", "
                                            << current_wrist_pose.pose.orientation.y << ", "
                                            << current_wrist_pose.pose.orientation.z << ", "
                                            << current_wrist_pose.pose.orientation.w);

            // Update trajectory marker
            geometry_msgs::Point p;
            p.x = current_wrist_pose.pose.position.x;
            p.y = current_wrist_pose.pose.position.y;
            p.z = current_wrist_pose.pose.position.z;
            line_strip.points.push_back(p);

            // Publish trajectory marker
            line_strip.header.stamp = ros::Time::now();
            marker_pub.publish(line_strip);

            // Publish arrival message
            std_msgs::String arrival_msg;
            arrival_msg.data = "End-effector has reached the target position.";
            arrival_pub.publish(arrival_msg);
        } else {
            ROS_WARN("Failed to plan to wrist position.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}