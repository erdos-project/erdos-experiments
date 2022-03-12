#include "latency/join.hpp"

#include "latency/msg/binary_data.hpp"
#include "latency/utils.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace latency {

Join::Join(const rclcpp::NodeOptions &options, const std::string &name)
    : Node(name, options) {
  std::string left_topic;
  this->declare_parameter<std::string>("left_topic", "left_topic");
  this->get_parameter("left_topic", left_topic);
  this->left_subscription = create_subscription<latency::msg::BinaryData>(
      left_topic, 1000, std::bind(&Join::on_left_msg, this, _1));

  std::string right_topic;
  this->declare_parameter<std::string>("right_topic", "right_topic");
  this->get_parameter("right_topic", right_topic);
  this->right_subscription = create_subscription<latency::msg::BinaryData>(
      right_topic, 1000, std::bind(&Join::on_right_msg, this, _1));

  std::string output_topic;
  this->declare_parameter<std::string>("output_topic", "output_topic");
  this->get_parameter("output_topic", output_topic);
  this->publisher = create_publisher<latency::msg::BinaryData>(output_topic, 1000);

  int64_t msg_size;
  this->declare_parameter<int64_t>("msg_size", 10000);
  this->get_parameter("msg_size", msg_size);

  srand(12052);
  for (size_t i = 0; i < (size_t)msg_size; i++) {
    data.push_back(rand() % 256);
  }

  RCLCPP_INFO(this->get_logger(), "joining %s and %s to %s with message size %ld",
              left_topic.c_str(), right_topic.c_str(), output_topic.c_str(), msg_size);
}

void Join::on_left_msg(const latency::msg::BinaryData::SharedPtr left_msg) {
  this->mu.lock();
  auto it = this->right_msgs.find(left_msg->timestamp);
  if (it == this->right_msgs.end()) {
    this->left_msgs[left_msg->timestamp] = left_msg;
  } else {
    auto output_msg = latency::msg::BinaryData();
    output_msg.timestamp = left_msg->timestamp;
    output_msg.data = this->data;
    if (left_msg->send_time_ns < it->second->send_time_ns) {
      output_msg.send_time_ns = left_msg->send_time_ns;
    } else {
      output_msg.send_time_ns = it->second->send_time_ns;
    }

    this->publisher->publish(output_msg);

    this->right_msgs.erase(left_msg->timestamp);

    if (left_msg->timestamp == UINT64_MAX) {
      rclcpp::shutdown();
    }
  }
  this->mu.unlock();
}

void Join::on_right_msg(const latency::msg::BinaryData::SharedPtr right_msg) {
  this->mu.lock();
  auto it = this->left_msgs.find(right_msg->timestamp);
  if (it == this->left_msgs.end()) {
    this->right_msgs[right_msg->timestamp] = right_msg;
  } else {
    auto output_msg = latency::msg::BinaryData();
    output_msg.timestamp = right_msg->timestamp;
    output_msg.data = this->data;
    if (right_msg->send_time_ns < it->second->send_time_ns) {
      output_msg.send_time_ns = right_msg->send_time_ns;
    } else {
      output_msg.send_time_ns = it->second->send_time_ns;
    }

    this->publisher->publish(output_msg);

    this->right_msgs.erase(right_msg->timestamp);

    if (right_msg->timestamp == UINT64_MAX) {
      rclcpp::shutdown();
    }
  }
  this->mu.unlock();
}

}  // namespace latency