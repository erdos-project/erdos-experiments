#include <iostream>

#include "latency/binary_source.hpp"
#include "latency/csv_sink.hpp"
#include "latency/join.hpp"
#include "latency/map.hpp"
#include "latency/receiver.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cout << "Please specify the number of pipelines" << std::endl;
  }

  // Uses all threads available on the machine.
  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  uint64_t num_pipelines = std::stoull(argv[1]);
  std::cout << "num_receivers: " << num_pipelines << std::endl;

  std::vector<latency::Map::SharedPtr> maps;
  std::vector<latency::Join::SharedPtr> joins;
  std::vector<latency::Receiver::SharedPtr> receivers;

  std::array<std::string, 8> map_names = {"camera_1",
                                          "camera_2",
                                          "lidar",
                                          "localization",
                                          "segmentation",
                                          "obstacle_detection",
                                          "traffic_light_detection",
                                          "lane_detection"};
  std::array<std::string, 8> join_names = {"tracker",    "lidar_traffic_light_fusion",
                                           "tmp_fusion", "fusion",
                                           "prediction", "tmp_planning",
                                           "planning",   "control"};

  for (size_t i = 0; i < num_pipelines; i++) {
    for (const auto &map_name : map_names) {
      std::string name = "pipeline_" + std::to_string(i) + "_" + map_name;
      auto map = std::make_shared<latency::Map>(options, name);
      exec.add_node(map);
      maps.push_back(map);
    }

    for (const auto &join_name : join_names) {
      std::string name = "pipeline_" + std::to_string(i) + "_" + join_name;
      auto join = std::make_shared<latency::Join>(options, name);
      exec.add_node(join);
      joins.push_back(join);
    }

    std::string receiver_name = "pipeline_" + std::to_string(i) + "_receiver";
    auto receiver = std::make_shared<latency::Receiver>(options, receiver_name);
    exec.add_node(receiver);
    receivers.push_back(receiver);
  }

  auto csv_sink = std::make_shared<latency::CSVSink>(options);
  exec.add_node(csv_sink);

  auto binary_source = std::make_shared<latency::BinarySource>(options);
  exec.add_node(binary_source);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
