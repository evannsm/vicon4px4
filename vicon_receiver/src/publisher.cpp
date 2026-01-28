#include "vicon_receiver/publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

Publisher::Publisher(std::string topic_name, rclcpp::Node *node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    // Pose Euler publisher with same topic name + "_euler" suffix
    rpy_publisher_ = node->create_publisher<vicon_receiver::msg::PoseEuler>(topic_name + "_euler", 10);

    // // PX4 odometry publisher with special QoS for PX4
    // rclcpp::QoS px4_qos(10);
    // px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    // px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    // px4_odom_publisher_ = node->create_publisher<px4_msgs::msg::VehicleOdometry>(
    //     "/fmu/in/vehicle_visual_odometry", px4_qos);

    is_ready = true;
}

void Publisher::publish(geometry_msgs::msg::PoseStamped pose_msg)
{
    // Publish PoseStamped (quaternion)
    position_publisher_->publish(pose_msg);

    // Convert quaternion to roll/pitch/yaw and publish as PoseEuler
    tf2::Quaternion q(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    vicon_receiver::msg::PoseEuler euler_msg;
    euler_msg.header = pose_msg.header;
    euler_msg.x = pose_msg.pose.position.x;
    euler_msg.y = pose_msg.pose.position.y;
    euler_msg.z = pose_msg.pose.position.z;
    euler_msg.roll = roll;
    euler_msg.pitch = pitch;
    euler_msg.yaw = yaw;
    rpy_publisher_->publish(euler_msg);

    // Publish PX4 VehicleOdometry
    // px4_msgs::msg::VehicleOdometry odom_msg;
    // px4_msgs::msg::VehicleOdometry odom_msg{};

    // odom_msg.timestamp = pose_msg.header.stamp.sec * 1000000ULL + pose_msg.header.stamp.nanosec / 1000ULL;
    // odom_msg.timestamp_sample = odom_msg.timestamp;
    // odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // odom_msg.position[0] = static_cast<float>(pose_msg.pose.position.x);
    // odom_msg.position[1] = static_cast<float>(pose_msg.pose.position.y);
    // odom_msg.position[2] = static_cast<float>(pose_msg.pose.position.z);

    // odom_msg.q[0] = static_cast<float>(pose_msg.pose.orientation.w);
    // odom_msg.q[1] = static_cast<float>(pose_msg.pose.orientation.x);
    // odom_msg.q[2] = static_cast<float>(pose_msg.pose.orientation.y);
    // odom_msg.q[3] = static_cast<float>(pose_msg.pose.orientation.z);

    // px4_odom_publisher_->publish(odom_msg);
}
