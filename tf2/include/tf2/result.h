#ifndef GEOMETRY2_RESULT_H
#define GEOMETRY2_RESULT_H

#include <cstddef>

class Result{
public:
  void incAbort(){
    localAbortCounts++;
  }

  uint64_t getAbortCount() const{
    return localAbortCounts;
  }

private:
  alignas(64) uint64_t localAbortCounts{};
};

#endif //GEOMETRY2_RESULT_H
