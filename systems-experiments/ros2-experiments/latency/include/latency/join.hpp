#ifndef LATENCY__Join_HPP_
#define LATENCY__Join_HPP_

#include <mutex>
#include <unordered_map>

#include "latency/msg/binary_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace latency {

class Join : public rclcpp::Node {
 public:
  explicit Join(const rclcpp::NodeOptions &options, const std::string &name = "Join");

 private:
  rclcpp::Subscription<latency::msg::BinaryData>::SharedPtr left_subscription;
  rclcpp::Subscription<latency::msg::BinaryData>::SharedPtr right_subscription;
  rclcpp::Publisher<latency::msg::BinaryData>::SharedPtr publisher;
  std::mutex mu;
  std::unordered_map<uint64_t, latency::msg::BinaryData::SharedPtr> left_msgs;
  std::unordered_map<uint64_t, latency::msg::BinaryData::SharedPtr> right_msgs;
  std::vector<uint8_t> data;
  void on_left_msg(const latency::msg::BinaryData::SharedPtr left_msg);
  void on_right_msg(const latency::msg::BinaryData::SharedPtr right_msg);
};

}  // namespace latency

#endif