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
        csv_file_.open("imu_data.csv");
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
            return;
        }
        csv_file_ << "Seconds,Nanoseconds,Accel_X,Accel_Y,Accel_Z\n"; // Write header
    }

    ~PlaybackNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close(); // Close the CSV file when done
        }
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

    void process_imu_data()
    {
        while (reader_->has_next())
        {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

            if (msg->topic_name != "/imu/data")
            {
                continue;
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

            // Apply the filter on the acceleration data
            apply_filter(ros_msg);

            // Write filtered data to the CSV file
            csv_file_ << ros_msg->header.stamp.sec << ","
                      << ros_msg->header.stamp.nanosec << ","
                      << filtered_acc_x_ << ","
                      << filtered_acc_y_ << ","
                      << filtered_acc_z_ << "\n";

            std::cout << "Filtered Accel x: " << ros_msg->linear_acceleration.x
                      << ", Accel y: " << ros_msg->linear_acceleration.y
                      << ", Accel z: " << ros_msg->linear_acceleration.z << "\n";
        }
    }

private:
    void apply_filter(const sensor_msgs::msg::Imu::SharedPtr &imu_msg)
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

    rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::ofstream csv_file_; // CSV file for storing filtered data

    double filtered_acc_x_, filtered_acc_y_, filtered_acc_z_;
    double previous_acc_x_ = 0.0, previous_acc_y_ = 0.0, previous_acc_z_ = 0.0;
    double previous_filtered_acc_x_ = 0.0, previous_filtered_acc_y_ = 0.0, previous_filtered_acc_z_ = 0.0;
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaybackNode>(argv[1]);
    node->process_imu_data();
    rclcpp::shutdown();

    return 0;
}
