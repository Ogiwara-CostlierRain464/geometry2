#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 5

class CCQueue{
private:
  std::atomic_int cur{-1};
  bool filled = false;
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};

public:
  void insert(const tf2::TransformStorage &e){
    // copy and insert.
    int insert_point = cur+1;
    for(int i = cur; i >= 0; i--){
      if(e.stamp_ < arr[i].stamp_){
        insert_point = i;
        break;
      }
    }
    if(insert_point == CC_ARR_SIZE){
      printf("ARR MAX CAPACITY\n");
      exit(0);
    }else{
      int i = cur;
      for(;;){
        arr[i+1] = arr[i];
        i--;
        if(i < insert_point){
          break;
        }
      }
    }

    arr[insert_point] = e;
    cur++;
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


    tf2::TransformStorage storage_target_time;
    storage_target_time.stamp_ = target;

    auto storage_it = std::lower_bound(
      arr.begin(),
      arr.begin() + cur,
      storage_target_time, std::greater<tf2::TransformStorage>());

    if(storage_it == arr.end()){
      printf("BBBBBB");
    }

    one = &*(storage_it);
    two = &*(++storage_it);

    // linear search
    // bound from up and down
//    tf2::TransformStorage *one_tmp, *two_tmp;
//    ros::Time one_t = ros::TIME_MIN, two_t = ros::TIME_MAX;
//
//    int seek_till = filled ? CC_ARR_SIZE : cur.load();
//
//    for(int i = 0; i <= seek_till; i++){
//      auto &time = arr[i].stamp_;
//      if(target < time){
//        if(time < two_t){
//          two_tmp = &arr[i];
//          two_t = time;
//        }
//      }else{ // target >= time
//        if(time > one_t){
//          one_tmp = &arr[i];
//          one_t = time;
//        }
//      }
//    }

  }

  void clear(){
    ;
  }
};


#endif //GEOMETRY2_CC_QUEUE_H
