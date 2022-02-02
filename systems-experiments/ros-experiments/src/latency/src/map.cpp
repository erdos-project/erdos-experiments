#include <chrono>
#include <cinttypes>
#include <iostream>

#include "latency/BinaryData.h"
#include "latency/LatencySample.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "utils.h"

class Map {
 public:
  Map(ros::Publisher output_topic, uint64_t msg_size) {
    this->output_topic = output_topic;
    for (int i = 0; i < msg_size; i++) {
      this->data.push_back(i);
    }
  }
  void callback(const latency::BinaryData::ConstPtr &msg) {
    latency::BinaryData output_msg;
    output_msg.send_time_ns = msg->send_time_ns;
    output_msg.timestamp = msg->timestamp;
    output_msg.data = this->data;

    this->output_topic.publish(output_msg);

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

 private:
  ros::Publisher output_topic;
  std::vector<uint8_t> data;
};

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cout
        << "Usage: rosrun latency receiver [name] [input_topic] [output_topic] [msg_size]"
        << std::endl;
  }

  std::string name(argv[1]);
  std::string input_topic(argv[2]);
  std::string output_topic(argv[3]);
  uint64_t msg_size = std::stoull(argv[4]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;
  ros::Publisher sample_pub = n.advertise<latency::BinaryData>(output_topic, 1000);

  Map map(sample_pub, msg_size);

  ros::Subscriber sub = n.subscribe(input_topic, 1000, &Map::callback, &map,
                                    ros::TransportHints().tcpNoDelay());
  ros::spin();

  return 0;
}
