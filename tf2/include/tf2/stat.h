#ifndef GEOMETRY2_STAT_H
#define GEOMETRY2_STAT_H

#include <vector>
#include <cstddef>
#include <chrono>
#include <ros/time.h>

namespace tf2 {

// Record start per thread.
struct Stat {
  std::vector<uint64_t> timestamps{};

  uint64_t getTimeStampsAve() const{
    uint64_t tmp{};
    for(auto &e: timestamps){
      tmp += e;
    }
    return tmp / timestamps.size();
  }
};

}
#endif //GEOMETRY2_STAT_H
