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

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(joint, 1'000, "Joint size");
DEFINE_uint64(iter, 1'000, "Iteration count");
DEFINE_double(read_ratio, 0.7, "read ratio, within [0,1]");
DEFINE_double(read_len, 1., "Percent of reading joint size, within [0,1]");
DEFINE_double(write_len, 1., "Number of reading joint size, within [0,1]");

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
  // link0 -- link1 -- .. -- link100
  for(size_t i = 0; i < FLAGS_joint; i++){
    auto i_iplus1 = trans("link" + to_string(i), "link" + to_string(i+1), 0.0001);
    bfc.setTransform(i_iplus1, "me");
  }
}

int64_t r_w_old(){
  OldBufferCore bfc{};
  make_snake(bfc);
  atomic_bool wait{true};
  vector<thread> threads{};

  size_t read_threads = ceil((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;
  size_t read_len = ceil((double) FLAGS_joint * FLAGS_read_len);
  size_t write_len = ceil((double) FLAGS_joint * FLAGS_write_len);

  for(size_t i = 0; i < read_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % FLAGS_joint;
        auto until = link + read_len;
        if(until > FLAGS_joint) until = FLAGS_joint;
        bfc.lookupTransform("link" + to_string(link),
                            "link" + to_string(until),
                            ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < write_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        int link = rand() % FLAGS_joint;
        auto until = link + write_len;
        if(until > FLAGS_joint) until = FLAGS_joint;
        for(size_t j = link; j < until; j++){
          bfc.setTransform(trans("link" + to_string(j),
                              "link" + to_string(j+1),
                              (double) i * 0.001), "me");
        }
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

int64_t r_w_alt(){
  BufferCore bfc{};
  make_snake(bfc);
  atomic_bool wait{true};
  vector<thread> threads{};

  size_t read_threads = ceil((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;
  size_t read_len = ceil((double) FLAGS_joint * FLAGS_read_len);
  size_t write_len = ceil((double) FLAGS_joint * FLAGS_write_len);

  for(size_t i = 0; i < read_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % FLAGS_joint;
        auto until = link + read_len;
        if(until > FLAGS_joint) until = FLAGS_joint;
        bfc.lookupTransform("link" + to_string(link),
                            "link" + to_string(until),
                            ros::Time(0));
      }
    });
  }

  for(size_t i = 0; i < write_threads; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        int link = rand() % FLAGS_joint;
        auto until = link + write_len;
        if(until > FLAGS_joint) until = FLAGS_joint;
        vector<TransformStamped> vec{};
        for(size_t j = link; j < until; j++){
          vec.push_back(trans("link" + to_string(j),
                              "link" + to_string(j+1),
                              (double) i * 0.001));
        }
        bfc.setTransforms(vec, "me");
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
  size_t operation_count = FLAGS_thread * FLAGS_iter;
  return ((double) operation_count) * (1000'000. / (double ) time);
}

void r_w_test(){
  auto time = r_w_old();
  std::cout << "Old tf r_w: " << throughput(time) << "req/sec\n";
  time = r_w_alt();
  std::cout << "Alt tf r_w: " << throughput(time) << "req/sec\n";
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  CONSOLE_BRIDGE_logInform("thread: %d", FLAGS_thread);
  CONSOLE_BRIDGE_logInform("joint: %d", FLAGS_joint);
  CONSOLE_BRIDGE_logInform("iter: %d", FLAGS_iter);
  CONSOLE_BRIDGE_logInform("read ratio: %lf", FLAGS_read_ratio);
  if(!(0. <= FLAGS_read_ratio and FLAGS_read_ratio <= 1.)){
    CONSOLE_BRIDGE_logError("wrong read ratio");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("read len: %lf", FLAGS_read_len);
  if(!(0. <= FLAGS_read_len and FLAGS_read_len <= 1.)){
    CONSOLE_BRIDGE_logError("wrong read len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("write len: %lf", FLAGS_write_len);
  if(!(0. <= FLAGS_write_len and FLAGS_write_len <= 1.)){
    CONSOLE_BRIDGE_logError("wrong write len");
    exit(-1);
  }

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  r_w_test();
  return 0;
}