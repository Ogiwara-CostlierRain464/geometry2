#include <iostream>
#include <thread>
#include <geometry_msgs/TransformStamped.h>
#include <atomic>
#include <chrono>
#include <vector>
#include <gflags/gflags.h>
#include <cstdlib>
#include <console_bridge/console.h>
#include <bits/stdc++.h>
#include <tbb/concurrent_vector.h>

#include "../old_tf2/old_buffer_core.h"
#include "tf2/buffer_core.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread_count, std::thread::hardware_concurrency(), "thread_count");
DEFINE_uint64(joint_count, 1000, "joint_count");
DEFINE_uint64(iter_count, 10'000, "iter_count");
DEFINE_double(read_ratio, 0.7, "read ratio [0,1]");
DEFINE_uint64(read_joints, 10, "Number of reading joint count");

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
        size_t link = rand() % FLAGS_joint_count;
        auto until = link + FLAGS_read_joints;
        if(until > FLAGS_joint_count) until = FLAGS_joint_count;
        bfc.lookupTransform("link" + to_string(link), "link" + to_string(until), when);
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

  size_t read_threads = ceil((double)FLAGS_thread_count * FLAGS_read_ratio);
  for(size_t i = 0; i < read_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        if(FLAGS_joint_count == FLAGS_read_joints){
          bfc.lookupTransform("head", "tail", ros::Time(0));
        }else{
          size_t link = rand() % FLAGS_joint_count;
          auto until = link + FLAGS_read_joints;
          if(until > FLAGS_joint_count) until = FLAGS_joint_count;
          bfc.lookupTransform("link" + to_string(link),
                              "link" + to_string(until),
                              ros::Time(0));
        }
      }
    });
  }

  size_t write_threads = FLAGS_thread_count - read_threads;
  for(size_t i = 0; i < write_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter_count; i++){
        int link = rand() % FLAGS_joint_count;
        bfc.setTransform(trans("link" + to_string(link),
                               "link" + to_string(link+1),
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

double throughput(int64_t time){
  size_t operation_count = std::thread::hardware_concurrency() * FLAGS_iter_count;
  return ((double) operation_count) * (1000'000. / (double ) time);
}

void r_w_test(){
  auto time = r_w<OldBufferCore>();
  std::cout << "Old tf r_w: " << throughput(time) << "req/sec\n";
  time = r_w<BufferCore>();
  std::cout << "Alt tf r_w: " << throughput(time) << "req/sec\n";
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  CONSOLE_BRIDGE_logInform("thread count: %d", FLAGS_thread_count);
  CONSOLE_BRIDGE_logInform("joint count: %d", FLAGS_joint_count);
  CONSOLE_BRIDGE_logInform("iter count: %d", FLAGS_iter_count);
  CONSOLE_BRIDGE_logInform("read ratio: %lf", FLAGS_read_ratio);
  if(!(0. <= FLAGS_read_ratio and FLAGS_read_ratio <= 1.)){
    CONSOLE_BRIDGE_logError("wrong read ratio param!");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("read joints: %d", FLAGS_read_joints);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  r_w_test();
  return 0;
}