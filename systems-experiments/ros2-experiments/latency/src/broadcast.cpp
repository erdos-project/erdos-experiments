#include <iostream>

#include "latency/binary_source.hpp"
#include "latency/csv_sink.hpp"
#include "latency/receiver.hpp"

/// Usage: ros2 run broadcast [msg_size] [num_receivers] [frequency] [output_filename]
/// [num_warmup_samples] [num_samples]
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cout << "Please specify the number of receivers" << std::endl;
  }

  // if (argc < 7) {
  //   std::cout << "Usage: ros2 run latency broadcast --ros-args -p num_msgs=[num_msgs]
  //   -p msg_size=[msg_size] [msg_size] [num_receivers] [frequency] "
  //                "[output_filename] [num_warmup_samples] [num_samples]"
  //             << std::endl;
  //   return 1;
  // }

  // Uses all threads available on the machine.
  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto binary_source = std::make_shared<latency::BinarySource>(options);
  exec.add_node(binary_source);

  uint64_t num_receivers = std::stoull(argv[1]);
  std::cout << "num_receivers: " << num_receivers << std::endl;

  std::vector<latency::Receiver::SharedPtr> receivers;
  for (size_t i = 0; i < num_receivers; i++) {
    auto receiver =
        std::make_shared<latency::Receiver>(options, "receiver_" + std::to_string(i));
    exec.add_node(receiver);
    receivers.push_back(receiver);
  }

  auto csv_sink = std::make_shared<latency::CSVSink>(options);
  exec.add_node(csv_sink);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
