#include "latency/csv_sink.hpp"

/// Usage: ros2 run broadcast [msg_size] [num_receivers] [frequency] [output_filename]
/// [num_warmup_samples] [num_samples]
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Use 4 threads.
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto csv_sink = std::make_shared<latency::CSVSink>(options);
  exec.add_node(csv_sink);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
