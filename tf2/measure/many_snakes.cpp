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
#include <bitset>

#include "tf2/stat.h"
#include "../old_tf2/old_buffer_core.h"
#include "tf2/buffer_core.h"
#include "xoroshiro128_plus.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using tf2::ReadStat;
using tf2::WriteStat;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(snake_num, 100, "Number of snakes");
DEFINE_uint64(snake_joints, 100, "Number of snake joints");
DEFINE_double(read_ratio, 0.5, "Read ratio, within [0,1]");
DEFINE_uint64(read_len, 100, "Read length in one snake, up to 100.");
DEFINE_uint64(write_len, 16, "Write length in one snake, up to 100.");
DEFINE_string(output, "/tmp/a.dat", "Output file");
DEFINE_string(only, "0010", "Bit representation of enabled methods. Silo, 2PL, Par, and Old from left to right bit.");
DEFINE_double(frequency, 0, "Frequency, when 0 then disabled");
DEFINE_uint64(loop_sec, 10, "Loop second");
DEFINE_bool(opposite_write_direction, true, "When true, opposite write direction");
DEFINE_bool(make_read_stat, true, "When true, make statistics. To enhance performance, turned off.");

using std::chrono::operator""s;
using std::chrono::duration_cast;

TransformStamped trans(
  const string &parent,
  const string &child,
  double sec){
  TransformStamped tr{};
  tr.header.frame_id = parent;
  tr.child_frame_id = child;
  tr.header.stamp = ros::Time(sec);
  tr.transform.rotation.w = 1;
  return tr;
}

template <typename T>
void make_tree(T &bfc){
  // map ----- link0-0 -- link0-2 -- ... link0-100
  //       |-- link1-0
  //       ...
  //       |-- link9999-0 -- link9999-100
  auto now = chrono::steady_clock::now();
  double sec = chrono::duration<double>(now.time_since_epoch()).count();

  for(size_t i = 0; i < FLAGS_snake_num; i++){
    auto i_map = trans("map", "link" + to_string(i) + "-0", sec);
    bfc.setTransform(i_map, "me");
    for(size_t j = 0; j < FLAGS_snake_joints; j++){
      auto edge = trans(
        "link" + to_string(i) + "-" + to_string(j),
        "link" + to_string(i) + "-" + to_string(j+1),
        sec);
      bfc.setTransform(edge, "me");
    }
  }
}

template <typename T>
struct BufferCoreWrapper{
  void init(){}
  void read(size_t snake, size_t from, size_t to, ReadStat *out_stat)const{}
  void write(size_t snake, size_t from, size_t to, double sec, WriteStat &out_stat, size_t &iter_acc){}
};

template <>
struct BufferCoreWrapper<OldBufferCore>{
  OldBufferCore bfc{};
  void init(){
    bfc.clear();
    make_tree(bfc);
  }
  void read(size_t snake, size_t from, size_t to, ReadStat *out_stat) const{
    // Read from bottom to top of the tree for better performance.
    assert(to > from);
    auto trans = bfc.lookupTransform(
      "link" + to_string(snake) + "-" + to_string(from),
      "link" + to_string(snake) + "-" + to_string(to),
      ros::Time(0));
    if(out_stat){
      out_stat->timestamps.push_back(trans.header.stamp.toNSec());
    }
  }
  void write(size_t snake, size_t from, size_t to, double sec, WriteStat &out_stat, size_t &iter_acc){
    // Write from bottom to top of the tree for better performance.
    assert(to > from);
    if(FLAGS_opposite_write_direction){ // Write from top to bottom
      for(size_t j = from; j < to; j++){
        bfc.setTransform(trans(
          "link" + to_string(snake) + "-" + to_string(j),
          "link" + to_string(snake) + "-" + to_string(j+1),
          sec), "me");
        iter_acc++;
      }
    }else{
      for(size_t j = to; j > from; j--){
        bfc.setTransform(trans(
          "link" + to_string(snake) + "-" + to_string(j-1),
          "link" + to_string(snake) + "-" + to_string(j),
          sec), "me");
        iter_acc++;
      }
    }
  }
};

enum AccessType{
  TF_Par, TF_2PL, TF_Silo
};

template <>
struct BufferCoreWrapper<BufferCore>{

  explicit BufferCoreWrapper(AccessType accessType_)
    : accessType(accessType_)
    , bfc(ros::Duration(100),
          1'020'000,
          accessType_ == TF_Silo ? tf2::Silo : tf2::TwoPhaseLock){}

  BufferCore bfc;
  AccessType accessType;

  void init(){
    bfc.clear();
    bfc.warmUpPages();
    make_tree(bfc);
  }
  void read(size_t snake, size_t from, size_t to, ReadStat *out_stat) const{
    if(accessType == TF_Par){
      assert(to > from);
      auto trans = bfc.lookupTransform(
        "link" + to_string(snake) + "-" + to_string(from),
        "link" + to_string(snake) + "-" + to_string(to),
        ros::Time(0), out_stat);
      if(out_stat){
        out_stat->timestamps.push_back(trans.header.stamp.toNSec());
      }
    }else{
      bfc.lookupLatestTransformXact(
        "link" + to_string(snake) + "-" + to_string(from),
        "link" + to_string(snake) + "-" + to_string(to),
        out_stat);
    }
  }
  void write(size_t snake, size_t from, size_t to, double sec, WriteStat &out_stat, size_t &iter_acc){
    assert(to > from);
    if(accessType == TF_Par){
      if(FLAGS_opposite_write_direction){
        for(size_t j = from; j < to; j++){
          bfc.setTransform(trans(
            "link" + to_string(snake) + "-" + to_string(j),
            "link" + to_string(snake) + "-" + to_string(j+1),
            sec), "me");
          iter_acc++;
        }
      }else{
        for(size_t j = to; j > from; j--){
          bfc.setTransform(trans(
            "link" + to_string(snake) + "-" + to_string(j-1),
            "link" + to_string(snake) + "-" + to_string(j),
            sec), "me");
          iter_acc++;
        }
      }
    }else{
      // which write direction is proper?
      vector<TransformStamped> vec{};
      if(FLAGS_opposite_write_direction){
        for(size_t j = from; j < to; j++){
          vec.push_back(trans(
            "link" + to_string(snake) + "-" + to_string(j),
            "link" + to_string(snake) + "-" + to_string(j+1),
            sec));
        }
      }else{
        for(size_t j = from; j > to; j--){
          vec.push_back(trans(
            "link" + to_string(snake) + "-" + to_string(j-1),
            "link" + to_string(snake) + "-" + to_string(j),
            sec));
        }
      }

      bfc.setTransformsXact(vec, "me", false, &out_stat);
      iter_acc++;
    }
  }
};

template <typename T>
T make_ave(const std::vector<T> &vec){
  // don't call this!
  assert(false);
}

template <>
double make_ave<double>(const std::vector<double> &vec){
  double tmp{};
  for(auto &e: vec) {
    tmp += e / (double) vec.size();
  }
  return tmp;
}

template <>
chrono::duration<double> make_ave<chrono::duration<double>>(const std::vector<chrono::duration<double>> &vec){
  chrono::duration<double> tmp{};
  for(auto &e: vec) {
    tmp += e / (double) vec.size();
  }
  return tmp;
}

template <typename T>
struct CountAccum{
  explicit CountAccum(size_t count): vec(count, T{}){}
  CountAccum(const CountAccum<T> &other) = delete;
  CountAccum(CountAccum<T> &&other) = delete;

  void record(size_t t_id, const T &data){
    vec[t_id] = data;
  };

  T average() const{
    return make_ave(vec);
  }

  T sum() const{
    return std::accumulate(vec.begin(), vec.end(), T{});
  }

  std::vector<T> vec;
};

struct RunResult{
  chrono::duration<double> time;
  double throughput;
  chrono::duration<double> readLatency;
  chrono::duration<double> writeLatency;
  chrono::duration<double> delay; // how far does took data from now
  // optional
  chrono::duration<double> var; // should be zero except latest
  double aborts; // aborts in write, should be zero expect latest
  double readAborts;

  double readThroughput;
  double writeThroughput;
};

double throughput(chrono::duration<double> time, size_t iter){
  return ((double) iter) * (1. / chrono::duration<double>(time).count());
}

// time, delay, latency, aborts, var
template <typename T>
RunResult run(BufferCoreWrapper<T> &bfc_w){
  auto read_threads = (size_t)std::round((double)FLAGS_thread * FLAGS_read_ratio);
  size_t write_threads = FLAGS_thread - read_threads;

  atomic_bool wait{true};
  vector<thread> threads{};

  CountAccum<double> throughput_acc_read_thread(read_threads);
  CountAccum<chrono::duration<double>> delay_acc_thread(read_threads);
  CountAccum<chrono::duration<double>> vars_acc_thread(read_threads);
  CountAccum<chrono::duration<double>> latencies_acc_read_thread(read_threads);
  CountAccum<double> read_wait_count(read_threads);
  CountAccum<double> deque_count_thread(read_threads);
  CountAccum<double> abort_acc_read_thread(read_threads);


  for(size_t t = 0; t < read_threads; t++){
    threads.emplace_back([t,&wait, &bfc_w, &delay_acc_thread,
                           &vars_acc_thread, &latencies_acc_read_thread,
                           &throughput_acc_read_thread, &read_wait_count,
                           &deque_count_thread,
                           &abort_acc_read_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());
      while (wait){;}
      chrono::duration<double> delay_iter_acc{}, var_iter_acc{}, latency_iter_acc{};
      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;
      size_t read_wait_count_acc{};
      size_t deque_count_acc{};
      uint64_t abort_iter_acc{};

      for(;;){
        ReadStat stat{};
        size_t snake = r.next() % FLAGS_snake_num;
        size_t from = r.next() % FLAGS_snake_joints;
        auto to = from + FLAGS_read_len;
        if(to > FLAGS_snake_joints) to = FLAGS_snake_joints;
        auto before = chrono::steady_clock::now();
        bfc_w.read(snake, from, to, FLAGS_make_read_stat ? &stat : nullptr);
        auto after = chrono::steady_clock::now();

        auto access_ave = operator""ns(stat.getTimeStampsAve());
        auto access_ave_sec =  chrono::duration<double>(access_ave).count();

        delay_iter_acc += before.time_since_epoch() - access_ave; // can be minus!
        var_iter_acc += operator""ns(stat.getTimeStampsStandardDiv());
        latency_iter_acc += after - before;
        read_wait_count_acc += stat.tryReadLockCount;
        deque_count_acc += stat.dequeSize;
        abort_iter_acc += stat.abortCount;

        if(FLAGS_frequency != 0){
          this_thread::sleep_for(operator""s((1. / FLAGS_frequency)));
        }

        iter_count++;

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      throughput_acc_read_thread.record(t, throughput(end_iter - start_iter, iter_count));
      delay_acc_thread.record(t,delay_iter_acc / (double) iter_count);
      vars_acc_thread.record(t, var_iter_acc / (double) iter_count);
      latencies_acc_read_thread.record(t, latency_iter_acc / (double) iter_count);
      read_wait_count.record(t, (double) read_wait_count_acc / (double) iter_count);
      deque_count_thread.record(t, (double) deque_count_acc / (double) iter_count);
      abort_acc_read_thread.record(t, (double) abort_iter_acc / (double) iter_count);
    });
  }

  CountAccum<double> abort_acc_thread(write_threads);
  CountAccum<double> throughput_acc_write_thread(write_threads);
  CountAccum<chrono::duration<double>> latencies_acc_write_thread(write_threads);

  for(size_t t = 0; t < write_threads; t++){
    threads.emplace_back([t, &bfc_w, &wait, &abort_acc_thread,
                           &throughput_acc_write_thread, &latencies_acc_write_thread, read_threads](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());
      while (wait){;}
      uint64_t abort_iter_acc{};
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;
      size_t iter_count = 0;
      chrono::duration<double> latency_iter_acc{};

      for(;;){
        size_t snake = r.next() % FLAGS_snake_num;
        size_t from = r.next() % FLAGS_snake_joints;
        auto to = from + FLAGS_write_len;
        if(to > FLAGS_snake_joints) to = FLAGS_snake_joints;
        vector<TransformStamped> vec{};
        auto before = chrono::steady_clock::now();
        double sec = chrono::duration<double>(before.time_since_epoch()).count(); // from sec
        WriteStat stat{};
        bfc_w.write(snake, from, to, sec, stat, iter_count);
        auto after = chrono::steady_clock::now();
        abort_iter_acc += stat.getAbortCount();
        latency_iter_acc += after - before;

        if(FLAGS_frequency != 0){
          this_thread::sleep_for(operator""s((1. / FLAGS_frequency)));
        }

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      throughput_acc_write_thread.record(t, throughput(end_iter - start_iter, iter_count));
      abort_acc_thread.record(t, (double) abort_iter_acc / (double) iter_count);
      latencies_acc_write_thread.record(t, latency_iter_acc / (double) iter_count);
    });
  }

  for(size_t t = 0; t < threads.size(); t++){
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(t, &cpuset);
    int rc = pthread_setaffinity_np(threads[t].native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
      std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
      exit(-1);
    }
  }

  bfc_w.init();
  asm volatile("" ::: "memory"); // force not to reorder.
  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }
  auto finish = chrono::high_resolution_clock::now();

  RunResult result{};
  result.time = finish - start;
  result.throughput = throughput_acc_read_thread.sum() + throughput_acc_write_thread.sum();
  result.readLatency = latencies_acc_read_thread.average();
  result.writeLatency = latencies_acc_write_thread.average();
  result.delay = delay_acc_thread.average();
  result.var = vars_acc_thread.average();
  result.aborts = abort_acc_thread.average();
  result.readThroughput = throughput_acc_read_thread.sum();
  result.writeThroughput = throughput_acc_write_thread.sum();
  result.readAborts = abort_acc_read_thread.average();

  cout << "read wait count: " << read_wait_count.average() << endl;
  cout << "deque size in snapshot: " << deque_count_thread.average() << endl;

  return result;
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  CONSOLE_BRIDGE_logInform("thread: %d", FLAGS_thread);
  CONSOLE_BRIDGE_logInform("snake_num: %d", FLAGS_snake_num);
  CONSOLE_BRIDGE_logInform("snake_joints: %d", FLAGS_snake_joints);
  CONSOLE_BRIDGE_logInform("read ratio: %lf", FLAGS_read_ratio);
  if(!(0. <= FLAGS_read_ratio and FLAGS_read_ratio <= 1.)){
    CONSOLE_BRIDGE_logError("wrong read ratio");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("read len: %d", FLAGS_read_len);
  if(!(0 <= FLAGS_read_len and FLAGS_read_len <= FLAGS_snake_joints)){
    CONSOLE_BRIDGE_logError("wrong read len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("write len: %d", FLAGS_write_len);
  if(!(0 <= FLAGS_write_len and FLAGS_write_len <= FLAGS_snake_joints)){
    CONSOLE_BRIDGE_logError("wrong write len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("Output: %s", FLAGS_output.c_str());
  CONSOLE_BRIDGE_logInform("Only: %s", FLAGS_only.c_str());
  CONSOLE_BRIDGE_logInform("frequency: %lf", FLAGS_frequency);
  CONSOLE_BRIDGE_logInform("Opposite write direction: %s", FLAGS_opposite_write_direction ? "true" : "false");
  CONSOLE_BRIDGE_logInform("Loop sec: %d", FLAGS_loop_sec);
  CONSOLE_BRIDGE_logInform("Make read stat: %s", FLAGS_make_read_stat ? "true" : "false");


  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  cout << std::setprecision(std::numeric_limits<double>::digits10);
  std::bitset<4> bs(FLAGS_only);

  RunResult old_result{};
  if(bs[0]){
    BufferCoreWrapper<OldBufferCore> bfc_w{};
    old_result = run(bfc_w);

    cout << "old: " << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(old_result.time).count() << "ms" << endl;
    cout << "\t" << "throughput: " << old_result.throughput << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(old_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(old_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "delay: " << chrono::duration<double, std::milli>(old_result.delay).count() << "ms" << endl;
  }

  RunResult par_result{};
  if(bs[1]){
    BufferCoreWrapper<BufferCore> bfc_w(AccessType::TF_Par);
    par_result = run(bfc_w);

    cout << "TF-Par:" << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(par_result.time).count() << "ms" << endl;
    cout << "\t" << "throughput: " << par_result.throughput << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(par_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(par_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "delay: " << chrono::duration<double, std::milli>(par_result.delay).count() << "ms" << endl;
  }

  RunResult _2pl_result{};
  if(bs[2]){
    BufferCoreWrapper<BufferCore> bfc_w(AccessType::TF_2PL);
    _2pl_result = run(bfc_w);

    cout << "TF-2PL:" << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(_2pl_result.time).count() << "ms" << endl;
    cout << "\t" << "throughput: " << _2pl_result.throughput << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(_2pl_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(_2pl_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "delay: " << chrono::duration<double, std::milli>(_2pl_result.delay).count() << "ms" << endl;
    cout << "\t" << "var: " << chrono::duration<double, std::milli>(_2pl_result.var).count() << "ms" << endl;
    cout << "\t" << "aborts: " << _2pl_result.aborts << " times" << endl;
  }

  RunResult silo_result{};
  if(bs[3]){
    BufferCoreWrapper<BufferCore> bfc_w(AccessType::TF_Silo);
    silo_result = run(bfc_w);

    cout << "TF-Silo:" << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(silo_result.time).count() << "ms" << endl;
    cout << "\t" << "throughput: " << silo_result.throughput << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(silo_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(silo_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "delay: " << chrono::duration<double, std::milli>(silo_result.delay).count() << "ms" << endl;
    cout << "\t" << "var: " << chrono::duration<double, std::milli>(silo_result.var).count() << "ms" << endl;
    cout << "\t" << "read aborts: " << silo_result.readAborts << " times" << endl;
  }

  if(FLAGS_frequency != 0){
    cout << "\033[31mWarn: frequency defined, so throughput is not making any sense!\033[0m" << endl;
  }
  if(!FLAGS_make_read_stat){
    cout << "\033[31mWarn: make read stat turned off, so `delay` and `var` are meaningless.\033[0m" << endl;
  }

  output << FLAGS_thread << " "; // 1
  output << FLAGS_snake_joints << " "; // 2
  output << FLAGS_read_ratio << " "; // 3
  output << FLAGS_read_len << " "; // 4
  output << FLAGS_write_len << " "; // 5
  output << FLAGS_frequency << " "; // 6
  output << old_result.throughput << " "; // 7
  output << par_result.throughput << " "; // 8
  output << _2pl_result.throughput << " "; // 9
  output << _2pl_result.aborts << " "; // 10
  output << chrono::duration<double, std::milli>(old_result.readLatency).count() << " "; // 11
  output << chrono::duration<double, std::milli>(par_result.readLatency).count() << " "; // 12
  output << chrono::duration<double, std::milli>(_2pl_result.readLatency).count() << " "; // 13
  output << chrono::duration<double, std::milli>(old_result.delay).count() << " "; // 14
  output << chrono::duration<double, std::milli>(par_result.delay).count() << " "; // 15
  output << chrono::duration<double, std::milli>(_2pl_result.delay).count() << " "; // 16
  output << chrono::duration<double, std::milli>(_2pl_result.var).count() << " "; // 17
  output << chrono::duration<double, std::milli>(old_result.writeLatency).count() << " "; // 18
  output << chrono::duration<double, std::milli>(par_result.writeLatency).count() << " "; // 19
  output << chrono::duration<double, std::milli>(_2pl_result.writeLatency).count() << " "; // 20
  output << old_result.readThroughput << " "; // 21
  output << par_result.readThroughput << " "; // 22
  output << _2pl_result.readThroughput << " "; // 23
  output << old_result.writeThroughput << " "; // 24
  output << par_result.writeThroughput << " "; // 25
  output << _2pl_result.writeThroughput << " "; // 26
  output << (FLAGS_opposite_write_direction ? "opposite" : "direct") << " "; // 27
  output << silo_result.throughput << " "; // 28
  output << silo_result.readAborts << " "; // 29
  output << chrono::duration<double, std::milli>(silo_result.readLatency).count() << " "; // 30
  output << chrono::duration<double, std::milli>(silo_result.writeLatency).count() << " "; // 31
  output << chrono::duration<double, std::milli>(silo_result.delay).count() << " "; // 32

  output << endl;
  output.close();

  // Fast exit, no need to wait de-alloc.
  exit(0);
}