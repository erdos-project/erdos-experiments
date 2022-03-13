#include "latency/join.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto join = std::make_shared<latency::Join>(options);
  exec.add_node(join);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
