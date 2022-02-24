#ifndef GEOMETRY2_WRITE_STAT_H
#define GEOMETRY2_WRITE_STAT_H

#include <cstddef>

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

#endif //GEOMETRY2_WRITE_STAT_H
