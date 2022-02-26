#ifndef GEOMETRY2_READ_CHECKER_H
#define GEOMETRY2_READ_CHECKER_H

#include <array>
#include <unordered_map>
#include "virtual_rwlock.h"

struct ReadChecker{
  std::array<VRWLock, 1'000'005> &mutexes;
  std::unordered_map<uint32_t, uint32_t> vHistory{}; // frame id -> version

  explicit ReadChecker(std::array<VRWLock, 1'000'005> &mutexes_)
  : mutexes(mutexes_){}

  void addRLock(uint32_t frame_id){
    auto v = mutexes.at(frame_id).virtualRLock();
    vHistory[frame_id] = v;
  }

  // how do we avoid seg fault?
  // w -> r : read thread wait for unlock
  // r -> w : write can be happening during reading. we need to avoid seg fault.
  // so how seg fault happen? when you check pointer!
  // so we need to use bare TransformStorage rather than std::deque.
  // we need memcpy for read and write.

  bool check(){
    // phase 2
    for(auto &pair: vHistory){
      auto frame_id = pair.first;
      auto old_v = pair.second;
      auto current_v = mutexes.at(frame_id).v.load(std::memory_order_acquire);
      if(current_v.locked or old_v != current_v.version){
        return false;
      }
    }
    return true;
  }

  void clear(){
    vHistory.clear();
  }
};

#endif //GEOMETRY2_READ_CHECKER_H
