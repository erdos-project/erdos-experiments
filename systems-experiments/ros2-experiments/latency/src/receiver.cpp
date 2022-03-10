#include "latency/receiver.hpp"

#include "latency/msg/binary_data.hpp"
#include "latency/msg/latency_sample.hpp"
#include "latency/utils.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace latency {

Receiver::Receiver(const rclcpp::NodeOptions &options, const std::string &name)
    : Node(name, options) {
  std::string recv_topic;
  this->declare_parameter<std::string>("recv_topic", "source");
  this->get_parameter("recv_topic", recv_topic);
  this->subscription = create_subscription<latency::msg::BinaryData>(
      recv_topic, 1000, std::bind(&Receiver::on_msg, this, _1));

  std::string sample_topic;
  this->declare_parameter<std::string>("sample_topic", "samples");
  this->get_parameter("sample_topic", sample_topic);
  this->publisher = create_publisher<latency::msg::LatencySample>(sample_topic, 1000);

  this->declare_parameter<bool>("is_inter_process", true);
  this->get_parameter("is_inter_process", this->is_inter_process);

  RCLCPP_INFO(this->get_logger(), "receiving on %s", recv_topic.c_str());
}

void Receiver::on_msg(const latency::msg::BinaryData::SharedPtr msg) const {
  uint64_t recv_time_ns = current_time_ns();

  auto sample = latency::msg::LatencySample();
  sample.timestamp = msg->timestamp;
  sample.latency_secs = ((double)(recv_time_ns - msg->send_time_ns)) / 1e9;
  sample.send_time_ns = msg->send_time_ns;
  sample.recv_time_ns = recv_time_ns;
  sample.msg_size = msg->data.size();
  sample.is_inter_process = this->is_inter_process;

  this->publisher->publish(sample);

  if (msg->timestamp >= UINT64_MAX) {
    rclcpp::shutdown();
  }
}

}  // namespace latency