#include <chrono>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <mutex>

#include "latency/LatencySample.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "utils.h"

class CSVSink {
 public:
  CSVSink(std::string filename, uint64_t num_warmup_samples, uint64_t num_samples) {
    this->file.open(filename);
    this->file << "latency_secs,send_time_ns,recv_time_ns,msg_size,is_inter_process"
               << std::endl;
    this->num_warmup_samples = num_warmup_samples;
    this->num_samples = num_samples;
  }
  ~CSVSink() {
    this->file.flush();
    this->file.close();
  }
  void record_sample(const latency::LatencySample::ConstPtr &msg) {
    if (msg->timestamp >= this->num_warmup_samples + this->num_samples) {
      ros::shutdown();
    } else if (msg->timestamp >= this->num_warmup_samples) {
      this->mu.lock();
      std::string is_inter_process = msg->is_inter_process ? "true" : "false";
      this->file << msg->latency_secs << "," << msg->send_time_ns << ","
                 << msg->recv_time_ns << "," << msg->msg_size << "," << is_inter_process
                 << std::endl;
      this->mu.unlock();
    }
  }

 private:
  std::mutex mu;
  std::ofstream file;
  uint64_t num_warmup_samples;
  uint64_t num_samples;
};

int main(int argc, char **argv) {
  if (argc < 6) {
    std::cout << "Usage: rosrun latency csv_sink [name] [topic] [filename] "
                 "[num-warmup-samples] [num-samples]"
              << std::endl;
    return 1;
  }
  std::string name(argv[1]);
  std::string topic(argv[2]);
  std::string filename(argv[3]);
  uint64_t num_warmup_samples = std::stoull(argv[4]);
  uint64_t num_samples = std::stoull(argv[5]);

  ros::init(argc, argv, name);
  ros::NodeHandle n;
  CSVSink csv_sink(filename, num_warmup_samples, num_samples);

  ros::Subscriber sub = n.subscribe(topic, 1000, &CSVSink::record_sample, &csv_sink,
                                    ros::TransportHints().tcpNoDelay());
  ros::spin();

  return 0;
}
