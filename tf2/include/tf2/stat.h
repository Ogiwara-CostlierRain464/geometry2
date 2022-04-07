#ifndef GEOMETRY2_STAT_H
#define GEOMETRY2_STAT_H

#include <cstddef>
#include <vector>
#include <cstddef>
#include <chrono>

namespace tf2{
  struct ReadStat {
    std::vector<uint64_t> timestamps{};
    size_t tryReadLockCount{};
    size_t dequeSize{};
    size_t abortCount{};

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

  class WriteStat{
  public:
    void incAbort(){
      localAbortCounts++;
    }

    uint64_t getAbortCount() const{
      return localAbortCounts;
    }

    alignas(64) uint64_t localAbortCounts{0};
    alignas(64) uint64_t tryWriteCount{0};
  };
}

#endif //GEOMETRY2_STAT_H
