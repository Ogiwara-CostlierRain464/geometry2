#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 5

class CCNode{
public:
  std::atomic_int cur{-1};
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
  CCNode firstNode{};
  CCNode *current{&firstNode};
  int fulledNodeNum = 0;
public:
  void insert(const tf2::TransformStorage &e){
//    bool node_changed = false;
//    current->copyAndInsert(e, node_changed);
//    if(node_changed){
//      fulledNodeNum++;
//      current = current->next;
//    }
    if(firstNode.cur == -1){
      firstNode.arr[0] = e;
      firstNode.cur = 0;
    }
  }

  bool empty() const{
    return firstNode.size() == 0;
  }

  int size() const{
    return CC_ARR_SIZE * fulledNodeNum + current->size();
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
    one = &firstNode.arr[0];
    two = &firstNode.arr[0];
  }

  void clear(){
    ;
  }
};


#endif //GEOMETRY2_CC_QUEUE_H
