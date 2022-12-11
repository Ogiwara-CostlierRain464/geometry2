#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 20

class CCQueue{
private:
  std::atomic_int cur{-1};
  bool filled = false;
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};

public:
  void insert(const tf2::TransformStorage &e){
    if(cur == CC_ARR_SIZE - 1){
      filled = true;
      arr[1] = e;
      cur = 1;
    }else{
      arr[cur+1] = e;
      cur++;
    }
  }

  bool empty() const{
    return cur == -1;
  }

  int size() const{
    if(filled){
      return CC_ARR_SIZE;
    }else{
      return cur+1;
    }
  }

  tf2::TransformStorage& front(){
    return latest();
  }

  tf2::TransformStorage& latest(){
    return arr[cur];
  }

  tf2::TransformStorage& first(){
    return arr[0];
  }

  void findTwoClose(const ros::Time &target,
                    tf2::TransformStorage* &one,
                    tf2::TransformStorage* &two){
    // linear search
    // bound from up and down
    tf2::TransformStorage *one_tmp, *two_tmp;
    ros::Time one_t = ros::TIME_MIN, two_t = ros::TIME_MAX;

    int seek_till = filled ? CC_ARR_SIZE : cur.load();

    for(int i = 0; i <= seek_till; i++){
      auto &time = arr[i].stamp_;
      if(target < time){
        if(time < two_t){
          two_tmp = &arr[i];
          two_t = time;
        }
      }else{ // target >= time
        if(time > one_t){
          one_tmp = &arr[i];
          one_t = time;
        }
      }
    }

    one = one_tmp;
    two = two_tmp;
  }

  void clear(){
    ;
  }
};


#endif //GEOMETRY2_CC_QUEUE_H
