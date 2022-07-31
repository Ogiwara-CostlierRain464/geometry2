#ifndef ALT_TF_RWLOCK_H
#define ALT_TF_RWLOCK_H

#include <atomic>
#include <tbb/concurrent_vector.h>

class RWLock {
public:
  alignas(64) std::atomic<int> counter;
  // counter == -1, write locked;
  // counter == 0, not locked;
  // counter > 0, there are $counter readers who acquires read-lock.

  RWLock(const RWLock &other) = delete;
  RWLock(RWLock &&other) = delete;
  RWLock() { counter.store(0, std::memory_order_release); }

  void init() { counter.store(0, std::memory_order_release); }

  void r_lock() {
    int expected, desired;
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != -1)
        desired = expected + 1;
      else {
        expected = counter.load(std::memory_order_acquire);
        continue;
      }

      if (counter.compare_exchange_strong(
        expected, desired,
        std::memory_order_acq_rel, std::memory_order_acquire))
        return;
    }
  }

  bool r_trylock() {
    int expected, desired;
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != -1)
        desired = expected + 1;
      else
        return false;

      if (counter.compare_exchange_strong(
        expected, desired,
        std::memory_order_acq_rel, std::memory_order_acquire))
        return true;
    }
  }

  void r_unlock() {
    counter--;
  }

  void w_lock() {
    int expected, desired(-1);
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != 0) {
        expected = counter.load(std::memory_order_acquire);
        continue;
      }
      if (counter.compare_exchange_strong(
        expected, desired,
        std::memory_order_acq_rel, std::memory_order_acquire))
        return;
    }
  }

  bool w_trylock() {
    int expected, desired(-1);
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != 0) return false;

      if (counter.compare_exchange_strong(
        expected, desired, std::memory_order_acq_rel, std::memory_order_acquire))
        return true;
    }
  }

  void w_unlock() { counter++; }

  // Upgrade, read -> write
  void upgrade() {
    int expected, desired(-1);
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != 1) { // only me is reading.
        expected = counter.load(std::memory_order_acquire);
        continue;
      }

      if (counter.compare_exchange_strong(
        expected, desired,
        std::memory_order_acquire, std::memory_order_acquire))
        return;
    }
  }

  bool tryupgrade() {
    int expected, desired(-1);
    expected = counter.load(std::memory_order_acquire);
    for (;;) {
      if (expected != 1) return false;

      if (counter.compare_exchange_strong(expected, desired,
                                          std::memory_order_acq_rel))
        return true;
    }
  }

  inline bool isLocked() const{
    return counter.load(std::memory_order_acquire) != 0;
  }
};

class ScopedWriteSetUnLocker{
public:
  explicit ScopedWriteSetUnLocker(RWLock* mutexes_)
    : mutexes(mutexes_){}

  void wLockIfNot(uint32_t id){
    // if not write locked, then add to write lock set.
    // upgrade is not allowed!
    if(writeLockedIdSet.find(id) == writeLockedIdSet.end()){
      if(readLockedIdSet.find(id) != readLockedIdSet.end()){
        // upgrade is not implemented!
        assert(false);
      }
      mutexes[id].w_lock();
      writeLockedIdSet.insert(id);
    }
  }

  bool tryWLockIfNot(uint32_t id){
    if(writeLockedIdSet.find(id) == writeLockedIdSet.end()){
      if(readLockedIdSet.find(id) != readLockedIdSet.end()){
        // upgrade is not implemented!
        assert(false);
      }
      bool result = mutexes[id].w_trylock();
      if(!result){
        return false;
      }
      writeLockedIdSet.insert(id);
    }
    // already locked, so success.
    return true;
  }

  void rLockIfNot(uint32_t id){
    // if not write locked, then add to read lock set.
    if(writeLockedIdSet.find(id) == writeLockedIdSet.end()){
      if(readLockedIdSet.find(id) == readLockedIdSet.end()){
        mutexes[id].r_lock();
        readLockedIdSet.insert(id);
      }
    }
  }

  void unlockAll(){
    for(auto id : writeLockedIdSet){
      mutexes[id].w_unlock();
    }
    for(auto id : readLockedIdSet){
      mutexes[id].r_unlock();
    }
    writeLockedIdSet.clear();
    readLockedIdSet.clear();
  }

  size_t wLockedSize() const{
    return writeLockedIdSet.size();
  }

  ~ScopedWriteSetUnLocker(){
    unlockAll();
  }
private:
  std::set<uint32_t> writeLockedIdSet{};
  std::set<uint32_t> readLockedIdSet{};
  RWLock* mutexes;
};

class ReadUnLocker{
public:
  explicit ReadUnLocker(RWLock &lock_)
    : lock(lock_){}

  inline void rLock(){
    lock.r_lock();
  }

  inline bool tryRLock(){
    return lock.r_trylock();
  }

  ~ReadUnLocker(){
    lock.r_unlock();
  }

private:
  RWLock &lock;
};

#endif //ALT_TF_RWLOCK_H
