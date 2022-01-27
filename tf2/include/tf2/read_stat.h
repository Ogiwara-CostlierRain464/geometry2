#ifndef GEOMETRY2_READ_STAT_H
#define GEOMETRY2_READ_STAT_H

#include <vector>
#include <cstddef>
#include <chrono>
#include <ros/time.h>

// Record start per thread.
struct ReadStat {
  std::vector<uint64_t> timestamps{};
  size_t tryReadLockCount{};
  size_t dequeSize{};

  uint64_t getTimeStampsAve() const{
    // don't sum!
    double tmp{0};
    for(auto &e: timestamps){
      tmp += (double) e / (double) timestamps.size();
    }
    return (size_t) tmp;
  }

  double getTimeStampsVar() const{
    if(timestamps.size() == 1){
      return 0;
    }

    auto ave = getTimeStampsAve();
    uint64_t tmp{0};
    for(auto &e: timestamps){
      tmp += (uint64_t) pow(e - ave, 2);
    }
    return  (double) tmp / (double) timestamps.size();
  }

  double getTimeStampsStandardDiv() const{
    return sqrt(getTimeStampsVar());
  }
};

#endif //GEOMETRY2_READ_STAT_H
