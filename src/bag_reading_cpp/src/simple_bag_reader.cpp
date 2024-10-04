#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/imu.hpp"  // Include IMU message type

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("playback_node")
    {
      // Change to publish IMU data
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

      // Timer to periodically read from the bag and publish messages
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));

      // Setup reader for rosbag
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
    }

  private:
    void timer_callback()
    {
      while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        // Adjust the topic to "/imu/data"
        if (msg->topic_name != "/imu/data") {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        
        // Change message type to IMU
        sensor_msgs::msg::Imu::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Imu>();

        // Deserialize the message
        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        // Publish the IMU message
        publisher_->publish(*ros_msg);

        // Output example (e.g., linear acceleration data)
        std::cout << "Accel x: " << ros_msg->linear_acceleration.x
                  << ", Accel y: " << ros_msg->linear_acceleration.y
                  << ", Accel z: " << ros_msg->linear_acceleration.z << "\n";

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    // Change serialization type to Imu
    rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}
