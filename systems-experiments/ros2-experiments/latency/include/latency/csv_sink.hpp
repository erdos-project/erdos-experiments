#ifndef LATENCY__CSV_SINK_HPP_
#define LATENCY__CSV_SINK_HPP_

#include <fstream>
#include <mutex>

#include "latency/msg/latency_sample.hpp"
#include "rclcpp/rclcpp.hpp"

namespace latency {

class CSVSink : public rclcpp::Node {
 public:
  explicit CSVSink(const rclcpp::NodeOptions &options);
  ~CSVSink();

 private:
  rclcpp::Subscription<latency::msg::LatencySample>::SharedPtr subscription;
  std::mutex mu;
  std::ofstream file;
  int64_t num_warmup_samples;
  int64_t num_samples;
  void on_msg(const latency::msg::LatencySample::SharedPtr msg);
};

}  // namespace latency

#endif