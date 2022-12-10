#ifndef GEOMETRY2_CC_QUEUE_H
#define GEOMETRY2_CC_QUEUE_H

#include <array>
#include <atomic>
#include "transform_storage.h"

#define CC_ARR_SIZE 50

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
    bool node_changed = false;
    current->copyAndInsert(e, node_changed);
    if(node_changed){
      fulledNodeNum++;
      current = current->next;
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

    // linear search
    // bound from up and down
    tf2::TransformStorage *one_tmp, *two_tmp;
    ros::Time one_t = ros::TIME_MIN, two_t = ros::TIME_MAX;

    CCNode* cur_node = &firstNode;
    for(;;){
      if(target > cur_node->latest().stamp_ ){
        assert(cur_node->next != nullptr);
        cur_node = cur_node->next;
        continue;
      }


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

      if(cur_node->next != nullptr){
        cur_node = cur_node->next;
      }else{
        // no next
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
