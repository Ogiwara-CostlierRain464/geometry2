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
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};

public:
  void insert(const tf2::TransformStorage &e){
    // copy and insert.
    int insert_point = cur+1;
    for(int i  = 0; i < cur; i++){
      if(e.stamp_ < arr[i].stamp_){
        insert_point = i;
        break;
      }
    }
    if(insert_point == CC_ARR_SIZE){
      printf("FAIL\n");
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
    return cur+1;
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

    auto it = std::lower_bound(
      arr.begin(),
      arr.begin() + cur,
      storage_target_time,
      std::greater<tf2::TransformStorage>());

    one = &*(it); //Older
    two = &*(++it); //Newer
  }

  void clear(){
    ;
  }
};


#endif //GEOMETRY2_CC_QUEUE_H
