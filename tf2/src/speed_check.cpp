#include <iostream>
#include <thread>
#include <geometry_msgs/TransformStamped.h>
#include <atomic>
#include <chrono>
#include <vector>

#include "../old_tf2/old_buffer_core.h"
#include "../include/tf2/buffer_core.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using namespace geometry_msgs;
using namespace std;

size_t THREAD_COUNT = 4;
size_t JOINT_COUNT = 1000;
size_t ITER_COUNT = 10'000;

TransformStamped trans(
  const string &parent,
  const string &child,
  double time){
  TransformStamped tr{};
  tr.header.frame_id = parent;
  tr.child_frame_id = child;
  tr.header.stamp = ros::Time(time);
  tr.transform.rotation.w = 1;
  return tr;
}

template <typename T>
void make_snake(T &bfc){
  // head -- link0 -- link1 -- .. -- link100 -- tail
  auto head_0 = trans("head", "link0", 1);
  bfc.setTransform(head_0, "me");
  for(size_t i = 0; i < JOINT_COUNT; i++){
    auto i_iplus1 = trans("link" + to_string(i), "link" + to_string(i+1), 1);
    bfc.setTransform(i_iplus1, "me");
  }
  auto max_tail = trans("link" + to_string(JOINT_COUNT), "tail", 1);
  bfc.setTransform(max_tail, "me");
}

template <typename T>
int64_t r_r(){
  T bfc{};
  make_snake(bfc);
  ros::Time when(1);

  atomic_bool wait{true};
  vector<thread> threads{};
  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER_COUNT; i++){
        bfc.lookupTransform("head", "tail", when);
      }
    });
  }

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads[i].join();
  }
  auto finish = chrono::high_resolution_clock::now();
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  return microseconds.count();
}

template <typename T>
int64_t r_w(){
  T bfc{};
  make_snake(bfc);

  atomic_bool wait{true};
  vector<thread> threads{};
  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER_COUNT; i++){
        bfc.lookupTransform("head", "tail", ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER_COUNT; i++){
        // access to another place is not utilized!
        bfc.setTransform(trans("head", "link0", (double) i * 0.001), "me");
      }
    });
  }

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }
  auto finish = chrono::high_resolution_clock::now();
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  return microseconds.count();
}

template <typename T>
int64_t w_w(){
  T bfc{};
  make_snake(bfc);

  atomic_bool wait{true};
  vector<thread> threads{};
  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER_COUNT; i++){
        bfc.lookupTransform("head", "tail", ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < THREAD_COUNT; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER_COUNT; i++){
        bfc.setTransform(trans("head", "link0", (double) i * 0.001), "me");
      }
    });
  }

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }
  auto finish = chrono::high_resolution_clock::now();
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  return microseconds.count();
}

void old_r_r(){
  auto time = r_r<OldBufferCore>();
  std::cout << "Old tf r_r: " << time << "µs\n";
}

void alt_r_r(){
  auto time = r_r<BufferCore>();
  std::cout << "Alt tf r_r: " << time << "µs\n";
}

void old_r_w(){
  auto time = r_w<OldBufferCore>();
  std::cout << "Old tf r_w: " << time << "µs\n";
}

void alt_r_w(){
  auto time = r_w<BufferCore>();
  std::cout << "Alt tf r_w: " << time << "µs\n";
}


int main(){
  alt_r_w();
  old_r_w();
  return 0;
}