#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <cstdio>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class ViconPX4Relay : public rclcpp::Node
{
public:
    ViconPX4Relay() : Node("vicon_px4_relay_node")
    {
        // Subscribe to Vicon pose topic
        vicon_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/drone/drone", 10,
            std::bind(&ViconPX4Relay::viconPoseCallback, this, _1));

        // Publish to PX4 visual odometry (default QoS)
        px4_visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        // Timer for 35Hz publishing (1000ms / 35 ≈ 28.57ms)
        timer_pub_ = this->create_wall_timer(
            std::chrono::milliseconds(28), // ~35Hz
            std::bind(&ViconPX4Relay::publishOdometry, this));

        RCLCPP_INFO(this->get_logger(), "Vicon PX4 Relay initialized - publishing at 35Hz");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_pub_;

    px4_msgs::msg::VehicleOdometry mocap_odometry_msg_{};
    bool has_received_pose_{false};

    void viconPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Frame & pose
        mocap_odometry_msg_.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        // Position
        const auto &p = msg->pose.position;
        mocap_odometry_msg_.position[0] = static_cast<float>(p.x);
        mocap_odometry_msg_.position[1] = static_cast<float>(p.y);
        mocap_odometry_msg_.position[2] = static_cast<float>(p.z);

        // Quaternion - PX4 order is [w, x, y, z]
        const auto &q = msg->pose.orientation;
        mocap_odometry_msg_.q[0] = static_cast<float>(q.w);
        mocap_odometry_msg_.q[1] = static_cast<float>(q.x);
        mocap_odometry_msg_.q[2] = static_cast<float>(q.y);
        mocap_odometry_msg_.q[3] = static_cast<float>(q.z);

        // Timestamp
        mocap_odometry_msg_.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
        mocap_odometry_msg_.timestamp_sample = mocap_odometry_msg_.timestamp;

        has_received_pose_ = true;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // Log once per second
            "Vicon pose received: [%.2f, %.2f, %.2f]",
            p.x, p.y, p.z);
    }

    void publishOdometry()
    {
        if (!has_received_pose_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000, // Warn every 2 seconds
                "No Vicon pose received yet, not publishing");
            return;
        }

        px4_visual_odom_pub_->publish(mocap_odometry_msg_);

        RCLCPP_DEBUG(this->get_logger(), "Published odometry at 35Hz");
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Vicon PX4 Relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconPX4Relay>());
    rclcpp::shutdown();
    return 0;
}