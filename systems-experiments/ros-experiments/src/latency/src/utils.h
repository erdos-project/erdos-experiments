#include <unistd.h>

#include <iostream>

#include "latency/BinaryData.h"

uint64_t current_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
