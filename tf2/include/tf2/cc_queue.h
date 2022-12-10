#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 20

class CCNode{
public:
  std::atomic_int cur{-1};
  std::array<tf2::TransformStorage, CC_ARR_SIZE> arr{};
  CCNode *next = nullptr;
  CCNode *back = nullptr;

  void insert(const tf2::TransformStorage &e){
    arr[cur+1] = e;
    cur++;
  }

  bool isFull() const{
    return size() == CC_ARR_SIZE;
  }

  int size() const{
    return cur+1;
  }

  tf2::TransformStorage& latest(){
    int id;
    ros::Time tmp = ros::TIME_MIN;
    int cur_snap = cur;
    for(int i = 0; i <= cur_snap; i++){
      if(arr[i].stamp_ >= tmp){
        id = i;
        tmp = arr[i].stamp_;
      }
    }

    return arr[id];
  }
};

class CCQueue{
private:
  CCNode firstNode{};
  CCNode *current = &firstNode;
  int fulledNodeNum = 0;
public:
  void insert(const tf2::TransformStorage &e){
    if(current->isFull()){
      auto next = new CCNode{};
      next->insert(e);
      next->back = current;
      current->next = next;
      // after new node is installed properly,
      // expose to read threads.
      // Note: write thread does not run concurrently,
      // So compare&swap is not required.
      current = next;
      fulledNodeNum++;
    }else{
      current->insert(e);
    }
  }

  bool empty() const{
    return firstNode.size() == 0;
  }

  int size() const{
    return 50 * fulledNodeNum + current->size();
  }

  tf2::TransformStorage& front(){
    return latest();
  }

  tf2::TransformStorage& latest(){
    return current->latest();
  }

  tf2::TransformStorage& first(){
    // First element is always the oldest data.
    return firstNode.arr[0];
  }

  void findTwoClose(const ros::Time &target,
                    tf2::TransformStorage* &one,
                    tf2::TransformStorage* &two){

    // linear search
    // bound from up and down
    tf2::TransformStorage *one_tmp, *two_tmp;
    ros::Time one_t = ros::TIME_MIN, two_t = ros::TIME_MAX;

    CCNode* cur_node = &firstNode;
    for(;;){
      for(int i = 0; i <= cur_node->cur; i++){
        auto &time = cur_node->arr[i].stamp_;
        if(target < time){
          if(time < two_t){
            two_tmp = &cur_node->arr[i];
            two_t = time;
          }
        }else{ // target >= time
          if(time > one_t){
            one_tmp = &cur_node->arr[i];
            one_t = time;
          }
        }
      }

      if(cur_node->next == nullptr){
        break;
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
