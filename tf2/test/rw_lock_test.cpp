#include <gtest/gtest.h>
#include "../include/tf2/rwlock.h"
#include "../include/tf2/buffer_core.h"

using namespace tf2;
using namespace std;

struct RWLockTest: public ::testing::Test{};

TEST_F(RWLockTest, read_unlocker){
  set<tf2::CompactFrameID> lock_set{};
  vector<RWLockPtr> mutex_;

  lock_set.insert(0); mutex_.emplace_back(std::move(make_unique<RWLock>()));
  lock_set.insert(1); mutex_.emplace_back(std::move(make_unique<RWLock>()));
  {
    ScopedReadSetUnLocker un_locker(lock_set, mutex_);
    mutex_[0]->r_lock(); mutex_[1]->r_lock();
  }

  EXPECT_FALSE(mutex_[0]->isLocked());
  EXPECT_FALSE(mutex_[1]->isLocked());
}

TEST_F(RWLockTest, write_unlocker){
  set<tf2::CompactFrameID> lock_set{};
  vector<RWLockPtr> mutex_;

  lock_set.insert(0); mutex_.emplace_back(std::move(make_unique<RWLock>()));
  lock_set.insert(1); mutex_.emplace_back(std::move(make_unique<RWLock>()));
  {
    ScopedWriteSetUnLocker un_locker(mutex_);
    un_locker.wLockIfNot(0);
    un_locker.wLockIfNot(1);
  }

  EXPECT_FALSE(mutex_[0]->isLocked());
  EXPECT_FALSE(mutex_[1]->isLocked());
}

TEST_F(RWLockTest, upgrade){
  RWLock lock{};
  lock.r_lock();
  bool upgraded = false;
  {
    UpdateUnLocker locker(lock, upgraded);
    lock.upgrade();
    upgraded = true;
  }
  EXPECT_FALSE(lock.isLocked());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
