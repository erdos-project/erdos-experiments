#ifndef LATENCY__RECEIVER_HPP_
#define LATENCY__RECEIVER_HPP_

#include "latency/msg/binary_data.hpp"
#include "latency/msg/latency_sample.hpp"
#include "rclcpp/rclcpp.hpp"

namespace latency {

class Receiver : public rclcpp::Node {
 public:
  explicit Receiver(const rclcpp::NodeOptions &options,
                    const std::string &name = "Receiver");

 private:
  rclcpp::Subscription<latency::msg::BinaryData>::SharedPtr subscription;
  rclcpp::Publisher<latency::msg::LatencySample>::SharedPtr publisher;
  bool is_inter_process;
  void on_msg(const latency::msg::BinaryData::SharedPtr msg) const;
};

}  // namespace latency

#endif