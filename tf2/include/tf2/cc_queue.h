#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

template <int S>
class CCQueue{
private:
  std::array<tf2::TransformStorage, S> arr{};
  std::atomic_int front_cnt{-1};

public:
  // may fail due to concurrent push
  bool tryPush(const tf2::TransformStorage &e){
    int expected = front_cnt.load(std::memory_order_acquire);
    int desired = expected+1;
    arr[desired] = e;
    if(front_cnt.compare_exchange_strong(
        expected, desired,
        std::memory_order_acq_rel, std::memory_order_acquire)){
      // `front` is now `desired`.
      // overwrite
      arr[desired] = e;
      return true;
    }else{
      return false; // fail to seek front
    }
  }

  bool empty() const{
    return front_cnt == -1;
  }

  int size() const{
    return front_cnt+1;
  }

  tf2::TransformStorage& front(){
    return arr[front_cnt];
  }

  tf2::TransformStorage& first(){
    return arr[0];
  }

  void findTwoClose(const ros::Time &target,
                    tf2::TransformStorage *one,
                    tf2::TransformStorage *two){
    // assert this is not empty.
    // assert at least two values,
    // and the target time is within the range
    for(int i = 0; i < front_cnt; i++){
      if(target >= arr[i].stamp_){
        one = &arr[i];
        two = &arr[i+1];
      }
    }
  }

  void clear(){
    front_cnt = -1;
  }
};

#endif //GEOMETRY2_CC_QUEUE_H
