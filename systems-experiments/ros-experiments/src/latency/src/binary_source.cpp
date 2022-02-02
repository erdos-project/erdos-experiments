#include <chrono>
#include <sstream>

#include "latency/BinaryData.h"
#include "ros/ros.h"
#include "utils.h"

void send_message(uint64_t msg_size, uint64_t timestamp, std::vector<uint8_t> &data,
                  ros::Publisher *publisher) {
  latency::BinaryData msg;
  msg.timestamp = timestamp;
  msg.data = data;

  msg.send_time_ns = current_time_ns();

  publisher->publish(msg);

  ros::spinOnce();
}

/// Usage:
/// rosrun binary_source sender [name] [topic] [msg-size] [num-warmup-msgs] [num-msgs]
/// [send-frequency]
int main(int argc, char **argv) {
  if (argc < 7) {
    std::cout << "Usage: rosrun latency binary_source [name] [topic] [msg-size] "
                 "[num-warmup-msgs] [num-msgs] [send-frequency]"
              << std::endl;

    return 1;
  }

  std::string name(argv[1]);
  std::string topic(argv[2]);
  uint64_t msg_size = std::stoull(argv[3]);
  uint64_t num_warmup_msgs = std::stoull(argv[4]);
  uint64_t num_msgs = std::stoull(argv[5]);
  double frequency = std::stod(argv[6]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;

  ros::Publisher latency_pub = n.advertise<latency::BinaryData>(topic, 1000);

  ros::Rate loop_rate(frequency);

  srand(420);

  std::vector<uint8_t> data;
  for (int i = 0; i < msg_size; i++) {
    data.push_back(rand() % 256);
  }

  for (uint64_t count = 0; ros::ok() && count < num_warmup_msgs + num_msgs; count++) {
    send_message(msg_size, count, data, &latency_pub);
    loop_rate.sleep();
  }

  // Send "top" watermark.
  send_message(msg_size, UINT64_MAX, data, &latency_pub);

  loop_rate.sleep();

  return 0;
}
