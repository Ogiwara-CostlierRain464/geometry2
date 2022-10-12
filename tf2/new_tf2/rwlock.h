#ifndef NEW_TF_RWLOCK_H
#define NEW_TF_RWLOCK_H

#include <atomic>
#include <set>
#include <cassert>

namespace new_tf2 {

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

    inline bool isLocked() const {
      return counter.load(std::memory_order_acquire) != 0;
    }
  };

  class ScopedUnLocker {
  public:

    void wLockIfNot(RWLock* lock) {
      // if not write locked, then add to write lock set.
      // upgrade is not allowed!
      if (writeLockSet.find(lock) == writeLockSet.end()) {
        if (readLockSet.find(lock) != readLockSet.end()) {
          // upgrade is not implemented!
          assert(false);
        }
        lock->w_lock();
        writeLockSet.insert(lock);
      }
    }

    bool tryWLockIfNot(RWLock* lock) {
      if (writeLockSet.find(lock) == writeLockSet.end()) {
        if (readLockSet.find(lock) != readLockSet.end()) {
          // upgrade is not implemented!
          assert(false);
        }
        bool result = lock->w_trylock();
        if (!result) {
          return false;
        }
        writeLockSet.insert(lock);
      }
      // already locked, so success.
      return true;
    }

    void rLockIfNot(RWLock *lock) {
      // if not write locked, then add to read lock set.
      if (writeLockSet.find(lock) == writeLockSet.end()) {
        if (readLockSet.find(lock) == readLockSet.end()) {
          lock->r_lock();
          readLockSet.insert(lock);
        }
      }
    }

    inline void unlockAll() {
      for (auto l: writeLockSet) {
        l->w_unlock();
      }
      for (auto l: readLockSet) {
        l->r_unlock();
      }
      writeLockSet.clear();
      readLockSet.clear();
    }

    size_t wLockedSize() const {
      return writeLockSet.size();
    }

    ~ScopedUnLocker() {
      unlockAll();
    }

  private:
    std::set<RWLock*> writeLockSet{};
    std::set<RWLock*> readLockSet{};
  };
}

#endif //NEW_TF_RWLOCK_H
