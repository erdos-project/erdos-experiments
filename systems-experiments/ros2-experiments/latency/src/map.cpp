#include "latency/map.hpp"

#include "latency/msg/binary_data.hpp"
#include "latency/utils.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace latency {

Map::Map(const rclcpp::NodeOptions &options, const std::string &name)
    : Node(name, options) {
  std::string input_topic;
  this->declare_parameter<std::string>("input_topic", "input_topic");
  this->get_parameter("input_topic", input_topic);
  this->subscription = create_subscription<latency::msg::BinaryData>(
      input_topic, 1000, std::bind(&Map::on_msg, this, _1));

  std::string output_topic;
  this->declare_parameter<std::string>("output_topic", "output_topic");
  this->get_parameter("output_topic", output_topic);
  this->publisher = create_publisher<latency::msg::BinaryData>(output_topic, 1000);

  this->declare_parameter<bool>("is_sensor", false);
  this->get_parameter("is_sensor", this->is_sensor);

  int64_t msg_size;
  this->declare_parameter<int64_t>("msg_size", 10000);
  this->get_parameter("msg_size", msg_size);

  srand(12052);
  for (size_t i = 0; i < (size_t)msg_size; i++) {
    data.push_back(rand() % 256);
  }

  RCLCPP_INFO(this->get_logger(), "mapping %s to %s with message size %ld",
              input_topic.c_str(), output_topic.c_str(), msg_size);
}

void Map::on_msg(const latency::msg::BinaryData::SharedPtr input_msg) const {
  auto output_msg = latency::msg::BinaryData();
  output_msg.timestamp = input_msg->timestamp;
  output_msg.data = this->data;
  if (this->is_sensor) {
    output_msg.send_time_ns = current_time_ns();
  } else {
    output_msg.send_time_ns = input_msg->send_time_ns;
  }

  this->publisher->publish(output_msg);

  if (input_msg->timestamp >= UINT64_MAX) {
    rclcpp::shutdown();
  }
}

}  // namespace latency