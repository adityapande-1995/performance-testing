#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

long unsigned int BUFFER_SIZE = 101;
double THROUGHPUT = 0;

std::map<std::string, int> msg_sizes {
  {"/k4a/points", 0},
  {"/k4a/depth/image", 0},
  {"/k4a/color/image", 0},
  {"/k4a/depth/camera_info", 0},
  {"/k4a/color/camera_info", 0},
};

std::map<std::string, std::vector<double>> current_timestamps {
  {"/k4a/points", {}},
  {"/k4a/depth/image", {}},
  {"/k4a/color/image", {}},
  {"/k4a/depth/camera_info", {}},
  {"/k4a/color/camera_info", {}},
};

template <typename T>
void msg_callback(const T msg, std::string topic, rclcpp::Time current_time) {

  if (current_timestamps[topic].size() == BUFFER_SIZE) {
    return;
  }

  auto logger = rclcpp::get_logger("subscriber_cpp");

  double current_nsec = current_time.nanoseconds();

  if (current_timestamps[topic].size() < BUFFER_SIZE) {
    current_timestamps[topic].push_back(current_nsec);
  }

  if (current_timestamps[topic].size() == BUFFER_SIZE) {
    int n = current_timestamps[topic].size();
    double data_flow = msg_sizes[topic] * (n) / (1024 * 1024);
    double delta_t = (current_timestamps[topic][n-1] - current_timestamps[topic][0])/ 1000000000 ;
    auto throughput = data_flow/delta_t;

    THROUGHPUT += throughput;

    std::string report =
      std::string("\n--------------") +
      std::string("\nFor topic: ") + topic +
      std::string("\nMsg size (bytes): ") + std::to_string(msg_sizes[topic]) +
      std::string("\nThroughput (MB/s): ") + std::to_string(throughput) +
      std::string("\nCumulative throughput (MB/s): ") + std::to_string(THROUGHPUT) +
      std::string("\n--------------");
    
      RCLCPP_INFO(logger, report.c_str());
  }

  // Find the size of the msg
  if (msg_sizes[topic] == 0) {
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&msg, &serialized_msg);
    msg_sizes[topic] = serialized_msg.size();
  }
}

class MinimalSubscriber : public rclcpp::Node
{
  public:
    // Subscriber objects
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_color;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_color_caminfo;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_depth_caminfo;

    MinimalSubscriber()
    : Node("subscriber_cpp")
    {
      // Start subscribers
      subscription_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/k4a/points", 10,
      [&](sensor_msgs::msg::PointCloud2 msg) {
        msg_callback<sensor_msgs::msg::PointCloud2>(msg, "/k4a/points", this->get_clock()->now());
      });

      subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
      "/k4a/depth/image", 10,
      [&](sensor_msgs::msg::Image msg) {
        msg_callback<sensor_msgs::msg::Image>(msg, "/k4a/depth/image", this->get_clock()->now());
      });

      subscription_color = this->create_subscription<sensor_msgs::msg::Image>(
      "/k4a/color/image", 10,
      [&](sensor_msgs::msg::Image msg) {
        msg_callback<sensor_msgs::msg::Image>(msg, "/k4a/color/image", this->get_clock()->now());
      });

      subscription_depth_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/k4a/depth/camera_info", 10,
      [&](sensor_msgs::msg::CameraInfo msg) {
        msg_callback<sensor_msgs::msg::CameraInfo>(msg, "/k4a/depth/camera_info", this->get_clock()->now());
      });

      subscription_color_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/k4a/color/camera_info", 10,
      [&](sensor_msgs::msg::CameraInfo msg) {
        msg_callback<sensor_msgs::msg::CameraInfo>(msg, "/k4a/color/camera_info", this->get_clock()->now());
      });
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
