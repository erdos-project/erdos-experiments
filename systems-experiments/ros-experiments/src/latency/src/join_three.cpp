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

class JoinThree {
 public:
  JoinThree(ros::Publisher output_topic, uint64_t msg_size) {
    this->output_topic = output_topic;
    for (int i = 0; i < msg_size; i++) {
      this->data.push_back(i);
    }
  }

  void left_callback(const latency::BinaryData::ConstPtr &msg) {
    auto it1 = this->right_msgs.find(msg->timestamp);
    auto it2 = this->center_msgs.find(msg->timestamp);
    if (it1 != this->right_msgs.end() && it2 != this->center_msgs.end()) {
      latency::BinaryData output_msg;
      output_msg.timestamp = msg->timestamp;
      output_msg.data = this->data;

      if (msg->send_time_ns < it1->second->send_time_ns &&
          msg->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = msg->send_time_ns;
      } else if (it1->second->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = it1->second->send_time_ns;
      } else {
        output_msg.send_time_ns = it2->second->send_time_ns;
      }

      this->output_topic.publish(output_msg);

      this->right_msgs.erase(msg->timestamp);
      this->center_msgs.erase(msg->timestamp);
    } else {
      this->left_msgs[msg->timestamp] = msg;
    }

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

  void right_callback(const latency::BinaryData::ConstPtr &msg) {
    auto it1 = this->left_msgs.find(msg->timestamp);
    auto it2 = this->center_msgs.find(msg->timestamp);
    if (it1 != this->left_msgs.end() && it2 != this->center_msgs.end()) {
      latency::BinaryData output_msg;
      output_msg.timestamp = msg->timestamp;
      output_msg.data = this->data;

      if (msg->send_time_ns < it1->second->send_time_ns &&
          msg->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = msg->send_time_ns;
      } else if (it1->second->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = it1->second->send_time_ns;
      } else {
        output_msg.send_time_ns = it2->second->send_time_ns;
      }

      this->output_topic.publish(output_msg);

      this->left_msgs.erase(msg->timestamp);
      this->center_msgs.erase(msg->timestamp);
    } else {
      this->right_msgs[msg->timestamp] = msg;
    }

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

  void center_callback(const latency::BinaryData::ConstPtr &msg) {
    auto it1 = this->left_msgs.find(msg->timestamp);
    auto it2 = this->right_msgs.find(msg->timestamp);
    if (it1 != this->left_msgs.end() && it2 != this->right_msgs.end()) {
      latency::BinaryData output_msg;
      output_msg.timestamp = msg->timestamp;
      output_msg.data = this->data;

      if (msg->send_time_ns < it1->second->send_time_ns &&
          msg->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = msg->send_time_ns;
      } else if (it1->second->send_time_ns < it2->second->send_time_ns) {
        output_msg.send_time_ns = it1->second->send_time_ns;
      } else {
        output_msg.send_time_ns = it2->second->send_time_ns;
      }

      this->output_topic.publish(output_msg);

      this->left_msgs.erase(msg->timestamp);
      this->right_msgs.erase(msg->timestamp);
    } else {
      this->center_msgs[msg->timestamp] = msg;
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
  std::unordered_map<uint64_t, latency::BinaryData::ConstPtr> center_msgs;
};

int main(int argc, char **argv) {
  if (argc < 7) {
    std::cout << "Usage: rosrun latency receiver [name] [left_topic] [center_topic] "
                 "[right_topic] [output_topic] [msg_size]"
              << std::endl;
  }

  std::string name(argv[1]);
  std::string left_topic(argv[2]);
  std::string right_topic(argv[3]);
  std::string center_topic(argv[4]);
  std::string output_topic(argv[5]);
  uint64_t msg_size = std::stoull(argv[6]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;
  ros::Publisher sample_pub = n.advertise<latency::BinaryData>(output_topic, 1000);

  JoinThree join_three(sample_pub, msg_size);

  ros::Subscriber left_sub = n.subscribe(left_topic, 1000, &JoinThree::left_callback,
                                         &join_three, ros::TransportHints().tcpNoDelay());
  ros::Subscriber center_sub =
      n.subscribe(right_topic, 1000, &JoinThree::center_callback, &join_three,
                  ros::TransportHints().tcpNoDelay());
  ros::Subscriber right_sub =
      n.subscribe(right_topic, 1000, &JoinThree::right_callback, &join_three,
                  ros::TransportHints().tcpNoDelay());

  ros::spin();

  return 0;
}
