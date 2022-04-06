#include <gtest/gtest.h>
#include <tbb/concurrent_vector.h>
#include "../include/tf2/rwlock.h"

using namespace std;

struct RWLockTest: public ::testing::Test{};

TEST_F(RWLockTest, read_unlocker){
  set<uint32_t> lock_set{};
  RWLock mutex_[2]{};
  {
    ScopedWriteSetUnLocker un_locker(mutex_);
    un_locker.rLockIfNot(0);
    un_locker.rLockIfNot(1);
  }

  EXPECT_FALSE(mutex_[0].isLocked());
  EXPECT_FALSE(mutex_[1].isLocked());
}

TEST_F(RWLockTest, write_unlocker){
  set<uint32_t> lock_set{};
  RWLock mutex_[2]{};
  {
    ScopedWriteSetUnLocker un_locker(mutex_);
    un_locker.wLockIfNot(0);
    un_locker.wLockIfNot(1);
  }

  EXPECT_FALSE(mutex_[0].isLocked());
  EXPECT_FALSE(mutex_[1].isLocked());
}

class LogPerNTimes{
public:
  void log(){
    if(count % 10 == 0){
      cout << "count is: " << count << endl;
    }
    count++;
  }
private:
  size_t count{0};
};

void n_times_func(){
  static LogPerNTimes logger{};
  logger.log();
}

TEST_F(RWLockTest, static_test){
  for(size_t i = 0; i < 100; i++){
    n_times_func();
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
