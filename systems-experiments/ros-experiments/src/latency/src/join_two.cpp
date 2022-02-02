#include <chrono>
#include <cinttypes>
#include <iostream>
#include <mutex>
#include <unordered_map>

#include "latency/BinaryData.h"
#include "latency/LatencySample.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "utils.h"

class JoinTwo {
 public:
  JoinTwo(ros::Publisher output_topic, uint64_t msg_size) {
    this->output_topic = output_topic;
    for (int i = 0; i < msg_size; i++) {
      this->data.push_back(i);
    }
  }

  void left_callback(const latency::BinaryData::ConstPtr &msg) {
    auto it = this->right_msgs.find(msg->timestamp);
    if (it == this->right_msgs.end()) {
      this->left_msgs[msg->timestamp] = msg;
    } else {
      latency::BinaryData output_msg;
      output_msg.timestamp = msg->timestamp;
      output_msg.data = this->data;
      if (msg->send_time_ns < it->second->send_time_ns) {
        output_msg.send_time_ns = msg->send_time_ns;
      } else {
        output_msg.send_time_ns = it->second->send_time_ns;
      }

      this->output_topic.publish(output_msg);

      this->right_msgs.erase(msg->timestamp);
    }

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

  void right_callback(const latency::BinaryData::ConstPtr &msg) {
    auto it = this->left_msgs.find(msg->timestamp);
    if (it == this->left_msgs.end()) {
      this->right_msgs[msg->timestamp] = msg;
    } else {
      latency::BinaryData output_msg;
      output_msg.timestamp = msg->timestamp;
      output_msg.data = this->data;

      if (msg->send_time_ns < it->second->send_time_ns) {
        output_msg.send_time_ns = msg->send_time_ns;
      } else {
        output_msg.send_time_ns = it->second->send_time_ns;
      }

      this->output_topic.publish(output_msg);

      this->left_msgs.erase(msg->timestamp);
    }

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

 private:
  ros::Publisher output_topic;
  std::vector<uint8_t> data;
  std::unordered_map<uint64_t, latency::BinaryData::ConstPtr> left_msgs;
  std::unordered_map<uint64_t, latency::BinaryData::ConstPtr> right_msgs;
};

int main(int argc, char **argv) {
  if (argc < 6) {
    std::cout << "Usage: rosrun latency receiver [name] [left_topic] [right_topic] "
                 "[output_topic] [msg_size]"
              << std::endl;
  }

  std::string name(argv[1]);
  std::string left_topic(argv[2]);
  std::string right_topic(argv[3]);
  std::string output_topic(argv[4]);
  uint64_t msg_size = std::stoull(argv[5]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;
  ros::Publisher sample_pub = n.advertise<latency::BinaryData>(output_topic, 1000);

  JoinTwo join_two(sample_pub, msg_size);

  ros::Subscriber left_sub = n.subscribe(left_topic, 1000, &JoinTwo::left_callback,
                                         &join_two, ros::TransportHints().tcpNoDelay());
  ros::Subscriber right_sub = n.subscribe(right_topic, 1000, &JoinTwo::right_callback,
                                          &join_two, ros::TransportHints().tcpNoDelay());

  ros::spin();

  return 0;
}
