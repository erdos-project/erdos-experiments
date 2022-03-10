#ifndef LATENCY__BINARY_SOURCE_HPP_
#define LATENCY__BINARY_SOURCE_HPP_

#include "latency/msg/binary_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace latency {

class BinarySource : public rclcpp::Node {
 public:
  explicit BinarySource(const rclcpp::NodeOptions &options);

 private:
  int64_t msg_size;
  int64_t num_msgs;
  uint64_t timestamp;
  std::vector<uint8_t> data;
  rclcpp::Publisher<latency::msg::BinaryData>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  void publish_message();
};

}  // namespace latency

#endif