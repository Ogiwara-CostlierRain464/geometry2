#include <iostream>
#include <thread>
#include <geometry_msgs/TransformStamped.h>
#include <atomic>
#include <chrono>
#include <vector>
#include <gflags/gflags.h>
#include <cstdlib>
#include <console_bridge/console.h>

#include "../old_tf2/old_buffer_core.h"
#include "../include/tf2/buffer_core.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread_count, std::thread::hardware_concurrency(), "thread_count");
DEFINE_uint64(joint_count, 1000, "joint_count");
DEFINE_uint64(iter_count, 10'000, "iter_count");

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
  for(size_t i = 0; i < FLAGS_joint_count; i++){
    auto i_iplus1 = trans("link" + to_string(i), "link" + to_string(i+1), 1);
    bfc.setTransform(i_iplus1, "me");
  }
  auto max_tail = trans("link" + to_string(FLAGS_joint_count), "tail", 1);
  bfc.setTransform(max_tail, "me");
}

template <typename T>
int64_t r_r(){
  T bfc{};
  make_snake(bfc);
  ros::Time when(1);

  atomic_bool wait{true};
  vector<thread> threads{};
  for(size_t i = 0; i < FLAGS_thread_count; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        bfc.lookupTransform("head", "tail", when);
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
int64_t r_w(){
  T bfc{};
  make_snake(bfc);

  atomic_bool wait{true};
  vector<thread> threads{};
  for(size_t i = 0; i < FLAGS_thread_count; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        bfc.lookupTransform("head", "tail", ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < FLAGS_thread_count; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        // access to another place is not utilized!
        int link = rand() % FLAGS_joint_count;
        bfc.setTransform(trans("link" + to_string(link), "link" + to_string(link+1),
                               (double) i * 0.001), "me");
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
  for(size_t i = 0; i < FLAGS_thread_count; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        bfc.lookupTransform("head", "tail", ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < FLAGS_thread_count; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
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


int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "thread count: " << FLAGS_thread_count << endl;
  cout << "joint count: " << FLAGS_joint_count << endl;
  cout << "iter count: " << FLAGS_iter_count << endl;

  alt_r_w();
  old_r_w();
  return 0;
}