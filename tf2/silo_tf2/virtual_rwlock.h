#ifndef SILO_TF_RWLOCK_H
#define SILO_TF_RWLOCK_H

#include <xmmintrin.h>
#include <atomic>

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
    v.store(copy_v);
  }
};

#endif