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
#include <fstream>
#include "../old_tf2/old_buffer_core.h"
#include "tf2/buffer_core.h"
#include "xoroshiro128_plus.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(joint, 100, "Joint size");
DEFINE_uint64(iter, 1'000, "Iteration count");
DEFINE_double(read_ratio, 0.5, "read ratio, within [0,1]");
DEFINE_uint64(read_len, 16, "Number of reading joint size ∈ [0, joint]");
DEFINE_uint64(write_len, 16, "Number of writing joint size ∈ [0, joint]");
DEFINE_string(output, "/tmp/a.dat", "Output file");
DEFINE_uint32(only, 0, "0: All, 1: Only snapshot, 2: Only Latest, 3: except old, 4: Only old");
DEFINE_bool(freshness, true, "Use freshness");

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
    auto i_iplus1 = trans("link" + to_string(i), "link" + to_string(i+1), 0.00001);
    bfc.setTransform(i_iplus1, "me");
  }
}

double r_w_old(OldBufferCore &bfc){
  int64_t time_acc{0};
  uint64_t delay_acc{};

  auto read_threads = (size_t)std::round((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;

  for(size_t count = 0; count < 6; count++){
    atomic_bool wait{true};
    vector<thread> threads{};
    vector<uint64_t> delays(read_threads, 0);

    for(size_t t = 0; t < read_threads; t++){
      threads.emplace_back([t, &wait, &bfc, &delays](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_read_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          auto trans = bfc.lookupTransform("link" + to_string(link),
                              "link" + to_string(until),
                              ros::Time(0));
          auto now = chrono::steady_clock::now();
          auto now_nano = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
          if(FLAGS_freshness){
            auto trans_nano = trans.header.stamp.toNSec();
            delays[t] += now_nano - trans_nano;
          }
        }
      });
    }

    for(size_t t = 0; t < write_threads; t++){
      threads.emplace_back([&](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_write_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          auto now = chrono::steady_clock::now();
          double nano = chrono::duration<double>(now.time_since_epoch()).count();
          for(size_t j = link; j < until; j++){
            bfc.setTransform(trans("link" + to_string(j),
                                   "link" + to_string(j+1),
                                   nano), "me");
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

    bfc.clear();
    make_snake(bfc);

    if(count == 0){ //warm up
      continue;
    }
    uint64_t delay_ave{};
    if(FLAGS_freshness){
      for(auto &d: delays){
        delay_ave += d;
      }
      delay_ave /= read_threads;
      delay_ave /= FLAGS_iter;
      delay_acc += delay_ave;
    }


    time_acc += microseconds.count();
  }
  ros::Time delay_ros{};
  delay_ros.fromNSec(delay_acc / 5);
  cout << "old delay: " << delay_ros << endl;

  return ((double) time_acc) / 5.0;
}

double r_w_alt(BufferCore &bfc){
  int64_t time_acc{0};
  uint64_t delay_acc{};

  auto read_threads = (size_t)std::round((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;

  for(size_t count = 0; count < 6; count++){
    atomic_bool wait{true};
    vector<thread> threads{};
    vector<uint64_t> delays(read_threads, 0);

    for(size_t t = 0; t < read_threads; t++){
      threads.emplace_back([t, &wait, &bfc, &delays](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_read_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          auto trans = bfc.lookupTransform("link" + to_string(link),
                              "link" + to_string(until),
                              ros::Time(0));
          auto now = chrono::steady_clock::now();
          auto now_nano = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
          if(FLAGS_freshness){
            auto trans_nano = trans.header.stamp.toNSec();
            delays[t] += now_nano - trans_nano;
          }
        }
      });
    }
    for(size_t t = 0; t < write_threads; t++){
      threads.emplace_back([&](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_write_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          auto now = chrono::steady_clock::now();
          double nano = chrono::duration<double>(now.time_since_epoch()).count();
          for(size_t j = link; j < until; j++){
            bfc.setTransform(trans("link" + to_string(j),
                                   "link" + to_string(j+1),
                                   nano), "me");
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

    bfc.clear();
    make_snake(bfc);

    if(count == 0){ //warm up
      continue;
    }
    uint64_t delay_ave{};
    if(FLAGS_freshness){
      for(auto &d: delays){
        delay_ave += d;
      }
      delay_ave /= read_threads;
      delay_ave /= FLAGS_iter;
      delay_acc += delay_ave;
    }
    time_acc += microseconds.count();
  }
  ros::Time delay_ros{};
  delay_ros.fromNSec(delay_acc / 5);
  cout << "snapshot delay: " << delay_ros << endl;

  return ((double) time_acc) / 5.;
}

// (time, aborts)
std::pair<double, double> r_w_trn(BufferCore &bfc){
  int64_t time_acc{0};
  size_t aborts_acc{0};
  uint64_t delay_acc{};

  auto read_threads = (size_t)std::round((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;

  for(size_t count = 0; count < 6; count++){
    atomic_bool wait{true};
    vector<thread> threads{};
    vector<uint64_t> delays(read_threads, 0);

    for(size_t t = 0; t < read_threads; t++){
      threads.emplace_back([t, &wait, &bfc, &delays](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          tf2::Stat stat{};
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_read_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          bfc.lookupLatestTransform("link" + to_string(link),
                                    "link" + to_string(until), FLAGS_freshness ? &stat : nullptr);
          auto now = chrono::steady_clock::now();
          auto now_nano = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
          if(FLAGS_freshness){
            // not all element has same timestamp!!
            auto access_ave_nano = stat.getTimeStampsAve();
            assert(now_nano > access_ave_nano);
            delays[t] += now_nano - access_ave_nano;
          }
        }
      });
    }

    std::vector<Result> results{};
    for(size_t t = 0; t < write_threads; t++){
      results.emplace_back();
    }

    for(size_t t = 0; t < write_threads; t++){
      threads.emplace_back([t, &bfc, &wait, &results](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_write_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          vector<TransformStamped> vec{};
          auto now = chrono::steady_clock::now();
          double nano = chrono::duration<double>(now.time_since_epoch()).count();
          for(size_t j = link; j < until; j++){
            vec.push_back(trans("link" + to_string(j),
                                "link" + to_string(j+1),
                                nano));
          }
          bfc.setTransforms(vec, "me", false, &results[t]);
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

    uint64_t abort_count{};
    for(auto &e: results){
      abort_count += e.getAbortCount();
    }

    bfc.clear();
    make_snake(bfc);

    if(count == 0){ // warm up
      continue;
    }
    uint64_t delay_ave{};
    if(FLAGS_freshness){
      for(auto &d: delays){
        delay_ave += d;
      }
      delay_ave /= read_threads;
      delay_ave /= FLAGS_iter;
      delay_acc += delay_ave;
    }

    time_acc += microseconds.count();
    aborts_acc += abort_count;
  }

  ros::Time delay_ros{};
  delay_ros.fromNSec(delay_acc / 5);
  cout << "latest delay: " << delay_ros << endl;

  return make_pair(((double) time_acc)/ 5., ((double) aborts_acc)/5.);
}

double throughput(double time){
  size_t operation_count = FLAGS_thread * FLAGS_iter;
  return ((double) operation_count) * (1000'000. / (double) time);
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
  CONSOLE_BRIDGE_logInform("read len: %d", FLAGS_read_len);
  if(!(0 <= FLAGS_read_len and FLAGS_read_len <= FLAGS_joint)){
    CONSOLE_BRIDGE_logError("wrong read len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("write len: %d", FLAGS_write_len);
  if(!(0 <= FLAGS_write_len and FLAGS_write_len <= FLAGS_joint)){
    CONSOLE_BRIDGE_logError("wrong write len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("Output: %s", FLAGS_output.c_str());
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  if(FLAGS_only != 0){
    CONSOLE_BRIDGE_logWarn("Only %s runs!!!", FLAGS_only == 1 ? "snapshot" : "latest");
  }

  double old_time = 0;
  if(FLAGS_only == 0 or FLAGS_only == 4){
    OldBufferCore bfc{};
    make_snake(bfc);
    old_time = r_w_old(bfc);
  }

  double alt_time = 0;
  if(FLAGS_only == 0 or FLAGS_only == 1 or FLAGS_only == 3){
    BufferCore bfc{};
    make_snake(bfc);
    alt_time = r_w_alt(bfc);
  }

  double trn_time = 0;
  double abort_count = 0;
  if(FLAGS_only == 0 or FLAGS_only == 2 or FLAGS_only == 3){
    BufferCore bfc{};
    make_snake(bfc);
    auto pair = r_w_trn(bfc);
    trn_time = pair.first;
    abort_count = pair.second;
  }

  cout << "Old time: " << old_time << endl;
  cout << "Snapshot time: " << alt_time << endl;
  cout << "Latest time: " << trn_time << endl;
  cout << "Aborts in latest: " << abort_count << endl;

  output << FLAGS_thread << " "; // 1
  output << FLAGS_joint << " "; // 2
  output << FLAGS_iter << " "; // 3
  output << FLAGS_read_ratio << " "; // 4
  output << FLAGS_read_len << " "; // 5
  output << FLAGS_write_len << " "; // 6
  output << throughput(old_time) << " "; // 7
  output << throughput(alt_time) << " "; // 8
  output << throughput(trn_time) << " "; // 9
  output << abort_count << endl;
  output.close();

  // data freshness?
  // how do we evaluate data freshness?
  // threads and freshness
  // old <-- --> fresh?
  /**
   * past           now
   * |-------------->|
   * |a|    |b|
   *
   * how old the data = now - old
   * so when small, then it is fresh!
   *
   * we should clear these conditions.
   * - what is freshness
   * - what is synchronous?
   * - how do we apply
   */
  return 0;
}