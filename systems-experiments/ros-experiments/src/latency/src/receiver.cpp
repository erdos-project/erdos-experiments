#include <chrono>
#include <cinttypes>
#include <iostream>

#include "latency/BinaryData.h"
#include "latency/LatencySample.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "utils.h"

class Receiver {
 public:
  Receiver(ros::Publisher sample_topic) { this->sample_topic = sample_topic; }
  void callback(const latency::BinaryData::ConstPtr &msg) {
    uint64_t recv_time_ns = current_time_ns();

    latency::LatencySample *sample = new latency::LatencySample();
    sample->timestamp = msg->timestamp;
    sample->latency_secs = ((double)(recv_time_ns - msg->send_time_ns)) / 1e9;
    sample->send_time_ns = msg->send_time_ns;
    sample->recv_time_ns = recv_time_ns;
    sample->msg_size = msg->data.size();
    sample->is_inter_process = false;

    this->sample_topic.publish(*sample);
    delete sample;

    if (msg->timestamp == UINT64_MAX) {
      ros::shutdown();
    }
  }

 private:
  ros::Publisher sample_topic;
};

/// Usage:
/// rosrun latency receiver [name] [recv_topic] [sample_topic]
int main(int argc, char **argv) {
  if (argc < 4) {
    std::cout << "Usage: rosrun latency receiver [name] [recv_topic] [sample_topic]"
              << std::endl;
  }

  std::string name(argv[1]);
  std::string recv_topic(argv[2]);
  std::string sample_topic(argv[3]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;
  ros::Publisher sample_pub = n.advertise<latency::LatencySample>(sample_topic, 1000);

  Receiver receiver(sample_pub);

  ros::Subscriber sub = n.subscribe(recv_topic, 1000, &Receiver::callback, &receiver,
                                    ros::TransportHints().tcpNoDelay());
  ros::spin();

  return 0;
}
