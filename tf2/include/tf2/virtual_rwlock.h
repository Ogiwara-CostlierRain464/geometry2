#ifndef SILO_TF_RWLOCK_H
#define SILO_TF_RWLOCK_H

#include <atomic>
#include <unordered_map>

struct Version{
  union {
    uint64_t body;
    struct {
      uint32_t version: 32;
      bool locked: 32;
    };
  };

  Version() noexcept
  : body{0}
  {}
};

struct VRWLock{
  std::atomic<Version> v{};

  uint32_t virtualRLock() const{
    // wait for w unlock, and get version.
    for(;;){
      auto expected = v.load(std::memory_order_acquire);
      if(expected.locked){
        continue;
      }else{
        // though it may be temporal, but write lock has been released!
        return expected.version;
      }
    }
  }

  void wLock(){
    // actually do lock
    for(;;){
      auto expected = v.load(std::memory_order_acquire);
      if(expected.locked){
        continue;
      }else{
        // now unlocked!
        auto desired = expected;
        expected.locked = false;
        desired.locked = true;
        if(v.compare_exchange_weak(expected, desired)){
          break;
        }
      }
    }
  }

  void wUnLock(){
    // actually unlock
    // if success, increment version
    auto copy_v = v.load(std::memory_order_acquire);
    copy_v.version++;
    copy_v.locked = false;
    v.store(copy_v);
  }
};

struct ReadChecker{
  VRWLock* mutexes;
  std::unordered_map<uint32_t, uint32_t> vHistory{}; // frame id -> version

  explicit ReadChecker(VRWLock* mutexes_)
    : mutexes(mutexes_){}

  void addRLock(uint32_t frame_id){
    if(vHistory.count(frame_id) != 0){
      // already r_locked
      return;
    }
    auto v = mutexes[frame_id].virtualRLock();
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
      auto current_v = mutexes[frame_id].v.load(std::memory_order_acquire);
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

#endif