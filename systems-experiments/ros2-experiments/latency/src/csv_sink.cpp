#include "latency/csv_sink.hpp"

#include <fstream>
#include <iostream>
#include <mutex>

#include "latency/msg/latency_sample.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace latency {

CSVSink::CSVSink(const rclcpp::NodeOptions &options) : Node("CSVSink", options) {
  std::string filename;
  this->declare_parameter<std::string>("filename", "out.csv");
  this->get_parameter("filename", filename);
  this->file.open(filename);
  this->file << "latency_secs,send_time_ns,recv_time_ns,msg_size,is_inter_process"
             << std::endl;

  this->declare_parameter<int64_t>("warmup_samples", 100);
  this->get_parameter("warmup_samples", this->num_warmup_samples);

  this->declare_parameter<int64_t>("samples", 10000);
  this->get_parameter("samples", this->num_samples);

  this->subscription = create_subscription<latency::msg::LatencySample>(
      "samples", 1000, std::bind(&CSVSink::on_msg, this, _1));

  RCLCPP_INFO(this->get_logger(),
              "writing to %s, %ld warmups excluded, %ld samples to write",
              filename.c_str(), this->num_warmup_samples, this->num_samples);
}

CSVSink::~CSVSink() {
  RCLCPP_INFO(this->get_logger(), "closing file");
  this->file.flush();
  this->file.close();
}

void CSVSink::on_msg(const latency::msg::LatencySample::SharedPtr msg) {
  if (msg->timestamp >= (uint64_t)(this->num_warmup_samples + this->num_samples)) {
    rclcpp::shutdown();
    return;
  } else if (msg->timestamp < (uint64_t)this->num_warmup_samples) {
    return;
  }

  this->mu.lock();
  std::string is_inter_process = msg->is_inter_process ? "true" : "false";
  this->file << msg->latency_secs << "," << msg->send_time_ns << "," << msg->recv_time_ns
             << "," << msg->msg_size << "," << is_inter_process << std::endl;
  this->mu.unlock();
}

}  // namespace latency