#include "latency/join.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions options;

  auto tmp_join = std::make_shared<latency::Join>(options, "tmp_join");
  exec.add_node(tmp_join);

  auto join = std::make_shared<latency::Join>(options, "join");
  exec.add_node(join);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
