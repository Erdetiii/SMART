#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
public:
    PlaybackNode()
        : Node("playback_node"), filtered_acc_x_(0.0), filtered_acc_y_(0.0), filtered_acc_z_(0.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("topic", 10);
        //timer_ = this->create_wall_timer(500ms, std::bind(&PlaybackNode::timer_callback, this));
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&PlaybackNode::topic_callback, this, std::placeholders::_1));
        msg_ = std::make_shared<sensor_msgs::msg::Imu>();
    }

    double high_pass_filter(double new_value, double previous_value, double previous_filtered_value, double alpha) {
        return alpha * (previous_filtered_value + new_value - previous_value);
    }

    double low_pass_filter(double new_value, double previous_value, double alpha) {
        return alpha * new_value + (1 - alpha) * previous_value;
    }

    double band_pass_filter(double new_value, double previous_value, double previous_filtered_value, double low_alpha, double high_alpha) {
        // Apply a high-pass filter on the input
        double high_pass_result = high_pass_filter(new_value, previous_value, previous_filtered_value, high_alpha);

        // Apply a low-pass filter on the high-pass result to get the band-pass effect
        return low_pass_filter(high_pass_result, previous_filtered_value, low_alpha);
    }

private:
    void apply_filter(sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // Simple moving average filter (you can modify this as needed)
        const double low_alpha = 0.1;  // Low-pass filter coefficient
        const double high_alpha = 0.7; // High-pass filter coefficient

        // Apply High-Pass Filter and store results directly in filtered_acc_x/y/z
        filtered_acc_x_ = band_pass_filter(imu_msg->linear_acceleration.x, previous_acc_x_, previous_filtered_acc_x_, low_alpha, high_alpha);
        filtered_acc_y_ = band_pass_filter(imu_msg->linear_acceleration.y, previous_acc_y_, previous_filtered_acc_y_, low_alpha, high_alpha);
        filtered_acc_z_ = band_pass_filter(imu_msg->linear_acceleration.z, previous_acc_z_, previous_filtered_acc_z_, low_alpha, high_alpha);

        // Update previous values for next iteration
        previous_acc_x_ = imu_msg->linear_acceleration.x;
        previous_acc_y_ = imu_msg->linear_acceleration.y;
        previous_acc_z_ = imu_msg->linear_acceleration.z;

        previous_filtered_acc_x_ = filtered_acc_x_;
        previous_filtered_acc_y_ = filtered_acc_y_;
        previous_filtered_acc_z_ = filtered_acc_z_;
    }

    void timer_callback()
    {
        if (msg_)
        {
            apply_filter(msg_);
 
            RCLCPP_INFO(this->get_logger(), "Publishing IMU data");
            publisher_->publish(*msg_);  // Publish the dereferenced message
        }
    }

    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        msg_ = imu_msg;

        RCLCPP_INFO(this->get_logger(), "Received IMU data");
        timer_callback();
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Imu::SharedPtr msg_;

    double filtered_acc_x_, filtered_acc_y_, filtered_acc_z_;
    double previous_acc_x_ = 0.0, previous_acc_y_ = 0.0, previous_acc_z_ = 0.0;
    double previous_filtered_acc_x_ = 0.0, previous_filtered_acc_y_ = 0.0, previous_filtered_acc_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaybackNode>();
    rclcpp::spin(node);  // Start spinning the node to process callbacks

    rclcpp::shutdown();
    return 0;
}
