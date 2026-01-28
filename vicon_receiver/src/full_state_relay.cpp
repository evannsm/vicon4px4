#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/full_state.hpp>
// #include <mocap4r2_msgs/msg/full_state.hpp>
// #include "vicon_receiver/msg/full_state.hpp"
// #include "vicon_receiver/msg/pose_euler.hpp"
#include "vicon_receiver/msg/full_state.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <cstdio>

// using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class FullStateRelay : public rclcpp::Node
{
public:
    FullStateRelay() : Node("full_state_relay_node")
    {
        // --- Tunables ---
        min_rate_hz_ = this->declare_parameter<double>("min_rate_hz", 50.0);               // gate threshold
        recent_timeout_sec_ = this->declare_parameter<double>("recent_timeout_sec", 0.10); // freshness window
        rate_ema_alpha_ = this->declare_parameter<double>("rate_ema_alpha", 0.9);          // 0.9 = slow, 0.5 = faster

        auto px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
        auto px4_qos_sub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

        vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", px4_qos_sub,
            std::bind(&FullStateRelay::vehicleOdometryCallback, this, _1));

        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4_qos_sub,
            std::bind(&FullStateRelay::localPositionCallback, this, _1));

        full_state_pub_ = create_publisher<vicon_receiver::msg::FullState>(
            "/merge_odom_localpos/full_state_relay", px4_qos_pub);

        timer_pub_full_state_ = this->create_wall_timer(
            25ms, std::bind(&FullStateRelay::publishFullState, this));
    }

private:
    // --- Subscriptions / publications ---
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Publisher<vicon_receiver::msg::FullState>::SharedPtr full_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_pub_full_state_;

    // --- Messages ---
    void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Update data
        full_state_msg_.position[0] = msg->position[0];
        full_state_msg_.position[1] = msg->position[1];
        full_state_msg_.position[2] = msg->position[2];

        full_state_msg_.velocity[0] = msg->velocity[0];
        full_state_msg_.velocity[1] = msg->velocity[1];
        full_state_msg_.velocity[2] = msg->velocity[2];

        full_state_msg_.q[0] = msg->q[0];
        full_state_msg_.q[1] = msg->q[1];
        full_state_msg_.q[2] = msg->q[2];
        full_state_msg_.q[3] = msg->q[3];

        full_state_msg_.angular_velocity[0] = msg->angular_velocity[0];
        full_state_msg_.angular_velocity[1] = msg->angular_velocity[1];
        full_state_msg_.angular_velocity[2] = msg->angular_velocity[2];

        // Measure & smooth callback rate
        const rclcpp::Time now = this->now();
        if (last_vo_cb_time_.nanoseconds() != 0)
        {
            const double dt = (now - last_vo_cb_time_).seconds();
            if (dt > 0.0)
            {
                const double inst_rate = 1.0 / dt;
                vo_rate_hz_ = (rate_ema_alpha_)*vo_rate_hz_ + (1.0 - rate_ema_alpha_) * inst_rate;
            }
        }
        last_vo_cb_time_ = now;
        have_vo_ = true;
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        full_state_msg_.acceleration[0] = msg->ax;
        full_state_msg_.acceleration[1] = msg->ay;
        full_state_msg_.acceleration[2] = msg->az;

        // Measure & smooth callback rate
        const rclcpp::Time now = this->now();
        if (last_lp_cb_time_.nanoseconds() != 0)
        {
            const double dt = (now - last_lp_cb_time_).seconds();
            if (dt > 0.0)
            {
                const double inst_rate = 1.0 / dt;
                lp_rate_hz_ = (rate_ema_alpha_)*lp_rate_hz_ + (1.0 - rate_ema_alpha_) * inst_rate;
            }
        }
        last_lp_cb_time_ = now;
        have_lp_ = true;
    }

    void publishFullState()
    {
        const rclcpp::Time now = this->now();

        // Freshness check (ensures stream hasn't stalled even if EMA remains high)
        const bool vo_fresh = have_vo_ && (now - last_vo_cb_time_).seconds() <= recent_timeout_sec_;
        const bool lp_fresh = have_lp_ && (now - last_lp_cb_time_).seconds() <= recent_timeout_sec_;

        const bool rates_ok = (vo_rate_hz_ >= min_rate_hz_) && (lp_rate_hz_ >= min_rate_hz_);
        const bool fresh_ok = vo_fresh && lp_fresh;

        if (rates_ok && fresh_ok)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500, // <-- one message at most every 500 ms (0.5 seconds)
                "Publishing FullState (rates: VO=%.1f Hz, LP=%.1f Hz): pos[%.2f, %.2f, %.2f]",
                vo_rate_hz_, lp_rate_hz_, full_state_msg_.position[0], full_state_msg_.position[1], full_state_msg_.position[2]);

            full_state_pub_->publish(full_state_msg_);
        }
        else
        {
            // Throttled warning so logs aren't spammy
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000, // <-- one message at most every 1000ms (1.0 seconds)
                "Not publishing: VO=%.1f Hz (fresh=%d), LP=%.1f Hz (fresh=%d), need >= %.1f Hz & fresh<=%.0f ms",
                vo_rate_hz_, vo_fresh, lp_rate_hz_, lp_fresh, min_rate_hz_, recent_timeout_sec_ * 1000.0);
        }
    }

    // --- State & gating params ---
    vicon_receiver::msg::FullState full_state_msg_{};

    rclcpp::Time last_vo_cb_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lp_cb_time_{0, 0, RCL_ROS_TIME};
    double vo_rate_hz_{0.0};
    double lp_rate_hz_{0.0};
    bool have_vo_{false};
    bool have_lp_{false};

    double min_rate_hz_{50.0};
    double recent_timeout_sec_{0.10};
    double rate_ema_alpha_{0.9};
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Full State Relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullStateRelay>());
    rclcpp::shutdown();
    return 0;
}