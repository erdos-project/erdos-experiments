#ifndef LATENCY__MAP_HPP_
#define LATENCY__MAP_HPP_

#include "latency/msg/binary_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace latency {

class Map : public rclcpp::Node {
 public:
  explicit Map(const rclcpp::NodeOptions &options, const std::string &name = "Map");

 private:
  rclcpp::Subscription<latency::msg::BinaryData>::SharedPtr subscription;
  rclcpp::Publisher<latency::msg::BinaryData>::SharedPtr publisher;
  bool is_sensor;
  std::vector<uint8_t> data;
  void on_msg(const latency::msg::BinaryData::SharedPtr msg) const;
};

}  // namespace latency

#endif