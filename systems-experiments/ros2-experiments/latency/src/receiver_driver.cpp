#include "latency/receiver.hpp"

/// Usage: ros2 run broadcast [msg_size] [num_receivers] [frequency] [output_filename]
/// [num_warmup_samples] [num_samples]
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Use 4 threads.
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto receiver = std::make_shared<latency::Receiver>(options);
  exec.add_node(receiver);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
