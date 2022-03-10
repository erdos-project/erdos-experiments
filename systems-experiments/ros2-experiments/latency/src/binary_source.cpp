#include "latency/binary_source.hpp"

#include <chrono>

#include "latency/msg/binary_data.hpp"
#include "latency/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace latency {

BinarySource::BinarySource(const rclcpp::NodeOptions &options)
    : Node("BinarySource", options), timestamp(0) {
  std::string topic;
  this->declare_parameter<std::string>("topic", "source");
  this->get_parameter("topic", topic);
  this->publisher = create_publisher<latency::msg::BinaryData>(topic, 1000);

  this->declare_parameter<int64_t>("msg_size", 10000);
  this->get_parameter("msg_size", this->msg_size);

  this->declare_parameter<int64_t>("num_msgs", 10100);
  this->get_parameter("num_msgs", this->num_msgs);

  double frequency;
  this->declare_parameter<double>("frequency", 30.0);
  this->get_parameter<double>("frequency", frequency);
  uint64_t period_ns = (uint64_t)(1e9 / frequency);

  srand(12052);
  for (size_t i = 0; i < (size_t)this->msg_size; i++) {
    data.push_back(rand() % 256);
  }

  RCLCPP_INFO(this->get_logger(), "sending %ld messages of size %ld @ %f Hz on %s",
              this->num_msgs, this->msg_size, frequency, topic.c_str());
  this->timer = create_wall_timer(std::chrono::nanoseconds(period_ns),
                                  std::bind(&BinarySource::publish_message, this));
}

void BinarySource::publish_message() {
  if (this->timestamp >= (uint64_t)this->num_msgs) {
    RCLCPP_INFO(this->get_logger(), "done sending messages");

    this->timer->cancel();

    // Send "top" watermark;
    auto message = latency::msg::BinaryData();
    message.timestamp = UINT64_MAX;
    message.data = this->data;
    message.send_time_ns = current_time_ns();
    this->publisher->publish(message);

    rclcpp::shutdown();

    return;
  }

  auto message = latency::msg::BinaryData();
  message.timestamp = this->timestamp;
  message.data = this->data;
  message.send_time_ns = current_time_ns();

  this->publisher->publish(message);

  this->timestamp++;
}

}  // namespace latency