#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

template <int S>
class CCQueue{
private:
  std::array<tf2::TransformStorage, S> arr{};
  std::atomic_int front_cnt{-1}; // what happen when exceeded?

public:

  void insert(const tf2::TransformStorage &e){
    arr[front_cnt+1] = e;
    front_cnt++;
  }

  bool empty() const{
    return front_cnt == -1;
  }

  int size() const{
    return front_cnt+1;
  }

  inline tf2::TransformStorage& front(){
    return latest();
  }

  tf2::TransformStorage& latest(){
    int id;
    ros::Time tmp = ros::TIME_MIN;
    int front_snap = front_cnt;
    for(int i = 0; i <= front_snap; i++){
      if(arr[i].stamp_ >= tmp){
        id = i;
        tmp = arr[i].stamp_;
      }
    }

    return arr[id];
  }



  tf2::TransformStorage& first(){
    // First element is always the oldest data.
    return arr[0];
  }

  void findTwoClose(const ros::Time &target,
                    tf2::TransformStorage* &one,
                    tf2::TransformStorage* &two){

    // linear search
    // bound from up and down
    int one_id, two_id;
    ros::Time one_t = ros::TIME_MIN, two_t = ros::TIME_MAX;

    int front_snap = front_cnt;
    for(int i = 0; i <= front_snap; i++){
      auto &time = arr[i].stamp_;
      if(target < time){
        if(time < two_t){
          two_id = i;
          two_t = time;
        }
      }else{ // target >= time
        if(time > one_t){
          one_id = i;
          one_t = time;
        }
      }
    }
    one = &arr[one_id];
    two = &arr[two_id];
  }

  void clear(){
    front_cnt = -1;
  }
};

#endif //GEOMETRY2_CC_QUEUE_H
