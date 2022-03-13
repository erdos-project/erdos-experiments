#include "latency/receiver.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto receiver = std::make_shared<latency::Receiver>(options);
  exec.add_node(receiver);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
