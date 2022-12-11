#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 20

class CCNode{
public:
  std::atomic_int cur{-1};
  bool filled = false;
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};
  CCNode *next{nullptr};

  void copyAndInsert(const tf2::TransformStorage &e, bool &node_changed){
    bool insert_middle = true;
    int insert_point = cur+1;
    for(int i = 0; i <= cur; i++){
      if(e.stamp_ < arr[i].stamp_){
        insert_point = i;
        break;
      }
    }

    if(insert_point == CC_ARR_SIZE){
      auto tmp = new CCNode{};
      tmp->arr[0] = e;
      tmp->cur = 0;
      next = tmp;
      node_changed = true;
    }else{
      assert(insert_middle);
      int i = cur;
      for(;;){
        if(i+1 == CC_ARR_SIZE){
          // need to move to next node!
          auto tmp = new CCNode{};
          tmp->arr[0] = arr[i];
          tmp->cur = 0;
          next = tmp;
          node_changed = true;
        }else{
          arr[i+1] = arr[i];
        }
        i--;
        if(i < insert_point){
          break;
        }
      }

      arr[insert_point] = e;
      cur++;
    }
  }

  bool isFull() const{
    return size() == CC_ARR_SIZE;
  }

  int size() const{
    return cur+1;
  }

  tf2::TransformStorage& latest(){
    return arr[cur];
  }
};

class CCQueue{
private:
  std::atomic_int cur{-1};
  bool filled = false;
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};

public:
  void insert(const tf2::TransformStorage &e){
    if(filled){
      if(cur == CC_ARR_SIZE - 1){
        arr[0] = e;
        cur = 0;
      }else{
        arr[cur+1] = e;
        cur++;
      }
    }else{
      if(cur == CC_ARR_SIZE - 1){
        filled = true;
        arr[0] = e;
        cur = 0;
      }else{
        arr[cur+1] = e;
        cur++;
      }
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
    if(!filled){
      return arr[0];
    }else{
      // linear search
      ros::Time tmp = ros::TIME_MAX;
      int id;
      for(int i = 0; i < CC_ARR_SIZE; i++){
        if(arr[i].stamp_ < tmp){
          tmp = arr[i].stamp_;
          id  = i;
        }
      }

      return arr[id];
    }
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
