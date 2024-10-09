#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
public:
    PlaybackNode(const std::string &bag_filename)
        : Node("playback_node"), filtered_acc_x_(0.0), filtered_acc_y_(0.0), filtered_acc_z_(0.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        timer_ = this->create_wall_timer(
            0.1ms, std::bind(&PlaybackNode::timer_callback, this));

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_filename;
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        if (!reader_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create a rosbag reader.");
            return;
        }

        reader_->open(storage_options);

        // Open CSV file for writing filtered data
        csv_file_.open("filtered_imu_data.csv");
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
            return;
        }
        csv_file_ << "Time,Accel_X,Accel_Y,Accel_Z\n"; // Write header
    }

    ~PlaybackNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close(); // Close the CSV file when done
        }
    }

private:
    void timer_callback()
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        if (msg->topic_name != "/imu/data")
        {
            return;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::Imu::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Imu>();
        try
        {
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Deserialization failed: %s", e.what());
            return;
        }

        // Apply a simple moving average filter (or any other filtering technique)
        apply_filter(ros_msg);

        publisher_->publish(*ros_msg);

        // Write filtered data to the CSV file
        csv_file_ << ros_msg->header.stamp.sec + ros_msg->header.stamp.nanosec / 1e9 << ","
                  << filtered_acc_x_ << ","
                  << filtered_acc_y_ << ","
                  << filtered_acc_z_ << "\n";

        std::cout << "Filtered Accel x: " << filtered_acc_x_
                  << ", Accel y: " << filtered_acc_y_
                  << ", Accel z: " << filtered_acc_z_ << "\n";

        return;
    }

    void apply_filter(const sensor_msgs::msg::Imu::SharedPtr &imu_msg)
    {
        // Simple moving average filter (you can modify this as needed)
        const double alpha = 0.1; // Filter coefficient (0 < alpha < 1)
        filtered_acc_x_ = alpha * imu_msg->linear_acceleration.x + (1 - alpha) * filtered_acc_x_;
        filtered_acc_y_ = alpha * imu_msg->linear_acceleration.y + (1 - alpha) * filtered_acc_y_;
        filtered_acc_z_ = alpha * imu_msg->linear_acceleration.z + (1 - alpha) * filtered_acc_z_;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::ofstream csv_file_; // CSV file for storing filtered data

    double filtered_acc_x_, filtered_acc_y_, filtered_acc_z_;
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
    rclcpp::shutdown();

    return 0;
}
