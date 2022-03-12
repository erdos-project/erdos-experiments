#include "latency/map.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto map = std::make_shared<latency::Map>(options);
  exec.add_node(map);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
