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
#include <limits>

#include "tf2/read_stat.h"
#include "../old_tf2/old_buffer_core.h"
#include "tf2/buffer_core.h"
#include "xoroshiro128_plus.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(joint, 100, "Joint size");
DEFINE_uint64(iter, 100, "Iteration count");
DEFINE_double(read_ratio, 0.5, "read ratio, within [0,1]");
DEFINE_uint64(read_len, 16, "Number of reading joint size ∈ [0, joint]");
DEFINE_uint64(write_len, 16, "Number of writing joint size ∈ [0, joint]");
DEFINE_string(output, "/tmp/a.dat", "Output file");
DEFINE_uint32(only, 0, "0: All, 1: Only snapshot, 2: Only Latest, 3: except old, 4: Only old");
DEFINE_double(frequency, 0, "frequency, when 0 then disabled");

constexpr size_t COUNT = 5;

using std::chrono::operator""s;
using std::chrono::duration_cast;

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
  auto now = chrono::steady_clock::now();
  double nano = chrono::duration<double>(now.time_since_epoch()).count();
  for(size_t i = 0; i < FLAGS_joint; i++){
    auto i_iplus1 = trans("link" + to_string(i),
                          "link" + to_string(i+1),
                          nano);
    bfc.setTransform(i_iplus1, "me");
  }
}

template <typename T>
struct BufferCoreWrapper{
  void init(){}
  void read(size_t link, size_t until, ReadStat &out_stat)const{}
  // return abort count
  void write(size_t link, size_t until, double nano_time, WriteStat &out_stat){}
};

template <>
struct BufferCoreWrapper<OldBufferCore>{
  OldBufferCore bfc{};
  void init(){
    bfc.clear();
    make_snake(bfc);
  }
  void read(size_t link, size_t until, ReadStat &out_stat) const{
    auto trans = bfc.lookupTransform("link" + to_string(link),
                                     "link" + to_string(until),
                                     ros::Time(0));
    out_stat.timestamps.push_back(trans.header.stamp.toNSec());
  }
  void write(size_t link, size_t until, double nano_time, WriteStat &out_stat){
    for(size_t j = link; j < until; j++){
      bfc.setTransform(trans("link" + to_string(j),
                             "link" + to_string(j+1),
                             nano_time), "me");
    }
  }
};

enum AccessType{
  Snapshot, Latest
};

template <>
struct BufferCoreWrapper<BufferCore>{

  explicit BufferCoreWrapper(AccessType accessType_)
  : accessType(accessType_){}

  BufferCore bfc{};
  AccessType accessType;

  void init(){
    bfc.clear();
    make_snake(bfc);
  }
  void read(size_t link, size_t until, ReadStat &out_stat) const{
    if(accessType == Snapshot){
      auto trans = bfc.lookupTransform("link" + to_string(link),
                                       "link" + to_string(until),
                                       ros::Time(0));
      out_stat.timestamps.push_back(trans.header.stamp.toNSec());
    }else{
      bfc.lookupLatestTransform("link" + to_string(link),
                                "link" + to_string(until), &out_stat);
    }
  }
  void write(size_t link, size_t until, double nano_time, WriteStat &out_stat){
    if(accessType == Snapshot){
      for(size_t j = link; j < until; j++){
        bfc.setTransform(trans("link" + to_string(j),
                               "link" + to_string(j+1),
                               nano_time), "me");
      }
    }else{
      vector<TransformStamped> vec{};
      for(size_t j = link; j < until; j++){
        vec.push_back(trans("link" + to_string(j),
                            "link" + to_string(j+1),
                            nano_time));
      }
      bfc.setTransforms(vec, "me", false, &out_stat);
    }
  }
};

template <typename T>
T make_ave(std::vector<T> &vec){
  T acc{};
  if(vec.empty()){
    return acc;
  }

  for(auto &e: vec){
    acc += e;
  }
  return acc / vec.size();
}

template <typename T>
struct CountAccum{
  explicit CountAccum(size_t count): vec(count, T{}){}

  void record(size_t t_id, const T &data){
    vec[t_id] = data;
  };

  T average(){
    return make_ave(vec);
  }

  std::vector<T> vec;
};

struct RunResult{
  chrono::duration<double> time; // per count.
  chrono::duration<double> latency;
  chrono::duration<double> delay; // how far does took data from now
  // optional
  chrono::duration<double> var; // should be zero except latest
  size_t aborts; // should be zero expect latest
};

// time, delay, latency, aborts, var
template <typename T>
RunResult run(BufferCoreWrapper<T> &bfc_w){
  CountAccum<chrono::duration<double>> time_acc(COUNT);
  CountAccum<size_t> aborts_acc(COUNT);
  CountAccum<chrono::duration<double>> delay_acc(COUNT);
  CountAccum<chrono::duration<double>> var_acc(COUNT);
  CountAccum<chrono::duration<double>> latency_acc(COUNT);

  auto read_threads = (size_t)std::round((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;

  for(size_t count = 0; count < COUNT+1; count++){
    atomic_bool wait{true};
    vector<thread> threads{};

    CountAccum<chrono::duration<double>> delay_acc_thread(read_threads);
    CountAccum<chrono::duration<double>> vars_acc_thread(read_threads);
    CountAccum<chrono::duration<double>> latencies_acc_thread(read_threads);

    for(size_t t = 0; t < read_threads; t++){
      threads.emplace_back([t, &wait, &bfc_w, &delay_acc_thread, &vars_acc_thread, &latencies_acc_thread](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}

        chrono::duration<double> delay_iter_acc{}, var_iter_acc{}, latency_iter_acc{};

        for(size_t i = 0; i < FLAGS_iter; i++){
          ReadStat stat{};
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_read_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          auto before = chrono::steady_clock::now();
          bfc_w.read(link, until, stat);
          auto now = chrono::steady_clock::now();

          auto access_ave = operator""ns(stat.getTimeStampsAve());
          assert(now.time_since_epoch() > access_ave);
          delay_iter_acc += now.time_since_epoch() - access_ave;
          var_iter_acc += operator""ns(stat.getTimeStampsStandardDiv());
          latency_iter_acc += now - before;

          if(FLAGS_frequency != 0){
            this_thread::sleep_for(operator""s((1 / FLAGS_frequency)));
          }
        }

        delay_acc_thread.record(t,delay_iter_acc / FLAGS_iter);
        vars_acc_thread.record(t, var_iter_acc / FLAGS_iter);
        latencies_acc_thread.record(t, latency_iter_acc / FLAGS_iter);
        // div by iter.
      });
    }

    CountAccum<size_t> abort_acc_thread(write_threads);

    for(size_t t = 0; t < write_threads; t++){
      threads.emplace_back([t, &bfc_w, &wait, &abort_acc_thread](){
        std::random_device rnd;
        Xoroshiro128Plus r(rnd());
        while (wait){;}
        uint64_t abort_iter_acc{};
        for(size_t i = 0; i < FLAGS_iter; i++){
          size_t link = r.next() % FLAGS_joint;
          auto until = link + FLAGS_write_len;
          if(until > FLAGS_joint) until = FLAGS_joint;
          vector<TransformStamped> vec{};
          auto now = chrono::steady_clock::now();
          double nano = chrono::duration<double>(now.time_since_epoch()).count();
          WriteStat stat{};
          bfc_w.write(link, until, nano, stat);
          abort_iter_acc += stat.getAbortCount();
        }
        abort_acc_thread.record(t, abort_iter_acc / FLAGS_iter);
      });
    }

    auto start = chrono::high_resolution_clock::now();
    wait = false;
    for(auto &e: threads){
      e.join();
    }
    auto finish = chrono::high_resolution_clock::now();

    bfc_w.init();

    if(count == 0){ // warm up
      continue;
    }

    time_acc.record(count - 1, finish - start);
    aborts_acc.record(count - 1, abort_acc_thread.average());
    delay_acc.record(count - 1, delay_acc_thread.average());
    var_acc.record(count - 1, vars_acc_thread.average());
    latency_acc.record(count - 1, latencies_acc_thread.average());
  }

  RunResult result{};
  result.time = time_acc.average();
  result.latency = latency_acc.average();
  result.delay = delay_acc.average();
  result.var = var_acc.average();
  result.aborts = aborts_acc.average();

  return result;
}

double throughput(chrono::duration<double> time){
  size_t operation_count = FLAGS_thread * FLAGS_iter;
  return ((double) operation_count) * (1. / chrono::duration<double>(time).count());
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
  CONSOLE_BRIDGE_logInform("Only: %d", FLAGS_only);
  CONSOLE_BRIDGE_logInform("frequency: %lf", FLAGS_frequency);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  RunResult old_result{};
  if(FLAGS_only == 0 or FLAGS_only == 4){
    BufferCoreWrapper<OldBufferCore> bfc_w{};
    bfc_w.init();
    old_result = run(bfc_w);
  }

  RunResult snapshot_result{};
  if(FLAGS_only == 0 or FLAGS_only == 1 or FLAGS_only == 3){
    BufferCoreWrapper<BufferCore> bfc_w(AccessType::Snapshot);
    bfc_w.init();
    snapshot_result = run(bfc_w);
  }

  RunResult latest_result{};
  if(FLAGS_only == 0 or FLAGS_only == 2 or FLAGS_only == 3){
    BufferCoreWrapper<BufferCore> bfc_w(AccessType::Latest);
    bfc_w.init();
    latest_result = run(bfc_w);
  }

  cout << std::setprecision(std::numeric_limits<double>::digits10);
  cout << "old: " << endl;
  cout << "\t" << "time: " << chrono::duration<double, std::milli>(old_result.time).count() << "ms" << endl;
  cout << "\t" << "latency: " << chrono::duration<double, std::milli>(old_result.latency).count() << "ms" << endl;
  cout << "\t" << "delay: " << chrono::duration<double, std::milli>(old_result.delay).count() << "ms" << endl;

  cout << "snapshot:" << endl;
  cout << "\t" << "time: " << chrono::duration<double, std::milli>(snapshot_result.time).count() << "ms" << endl;
  cout << "\t" << "latency: " << chrono::duration<double, std::milli>(snapshot_result.latency).count() << "ms" << endl;
  cout << "\t" << "delay: " << chrono::duration<double, std::milli>(snapshot_result.delay).count() << "ms" << endl;

  cout << "latest:" << endl;
  cout << "\t" << "time: " << chrono::duration<double, std::milli>(latest_result.time).count() << "ms" << endl;
  cout << "\t" << "latency: " << chrono::duration<double, std::milli>(latest_result.latency).count() << "ms" << endl;
  cout << "\t" << "delay: " << chrono::duration<double, std::milli>(latest_result.delay).count() << "ms" << endl;
  cout << "\t" << "var: " << chrono::duration<double, std::milli>(latest_result.var).count() << "ms" << endl;
  cout << "\t" << "aborts: " << latest_result.aborts << " times" << endl;

  if(FLAGS_frequency != 0){
    cout << "\033[31mWarn: frequency defined, so throughput is not making any sense!\033[0m" << endl;
  }

  output << FLAGS_thread << " "; // 1
  output << FLAGS_joint << " "; // 2
  output << FLAGS_iter << " "; // 3
  output << FLAGS_read_ratio << " "; // 4
  output << FLAGS_read_len << " "; // 5
  output << FLAGS_write_len << " "; // 6
  output << throughput(old_result.time) << " "; // 7
  output << throughput(snapshot_result.time) << " "; // 8
  output << throughput(latest_result.time) << " "; // 9
  output << latest_result.aborts << endl; // 10
  output.close();

  return 0;
}