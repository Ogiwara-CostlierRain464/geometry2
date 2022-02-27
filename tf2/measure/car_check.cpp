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
#include "../silo_tf2/silo_buffer_core.h"
#include "tf2/buffer_core.h"
#include "xoroshiro128_plus.h"

using old_tf2::OldBufferCore;
using tf2::BufferCore;
using silo_tf2::SiloBufferCore;
using namespace geometry_msgs;
using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(vehicle, 67, "Vehicle count per ");
DEFINE_double(read_ratio, 0.5, "read ratio, within [0,1]");
DEFINE_uint64(read_len, 67, "Number of reading vehicles size ∈ [0, vehicle]");
DEFINE_uint64(write_len, 1, "Number of writing vehicles size ∈ [0, vehicles]");
DEFINE_string(output, "/tmp/a.dat", "Output file");
DEFINE_double(frequency, 0, "frequency, when 0 then disabled");
DEFINE_uint64(loop_sec, 5, "loop second");
DEFINE_double(insert_span, 6, "new car arrive span in sec");
DEFINE_double(only, 3, "0: All, 1: old, 2: 2PL, 3: Silo");


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
void make_base_station(T &bfc){
// have to add new interface. read specified ids.
// what was TPCC workload?
// add one car per sec.
// but no delete.
// read all constantly.
// what is write?
// - new AuV arrived
// - update it's position.
  auto now = chrono::steady_clock::now();
  double nano = chrono::duration<double>(now.time_since_epoch()).count();

  for(size_t i = 0; i < FLAGS_vehicle; i++){
    auto map_i = trans("map",
                          "link" + to_string(i),
                          nano);
    bfc.setTransform(map_i, "me");
  }
}

template <typename T>
struct BufferCoreWrapper{
  void init(){}
  void read(size_t start)const{}
  void write(size_t id, double nano_time, size_t &iter_acc){}
};

template <>
struct BufferCoreWrapper<OldBufferCore>{
  OldBufferCore bfc{};
  void init(){
    bfc.clear();
    make_base_station(bfc);
  }
  void read(size_t start) const{
    // read from start to start+FLAGS_vehicle
    vector<string> frames{};
    for(size_t i = start; i < start + FLAGS_read_len; i++){
      frames.push_back("link" + to_string(i));
    }
    bfc.justReadFrames(frames);
  }

  void write(size_t id, double nano_time, size_t &iter_acc){
    bfc.setTransform(trans("map", "link" + to_string(id), nano_time), "me");
    iter_acc++;
  }
};

template <>
struct BufferCoreWrapper<BufferCore>{
  BufferCore bfc{};

  void init(){
    bfc.clear();
    make_base_station(bfc);
  }
  void read(size_t start) const{
    vector<string> frames{};
    for(size_t i = start; i < start + FLAGS_read_len; i++){
      frames.push_back("link" + to_string(i));
    }
    bfc.justReadFrames(frames);
  }

  void write(size_t id, double nano_time, size_t &iter_acc){
    vector<TransformStamped> vec{};
    vec.push_back(trans("map", "link" + to_string(id), nano_time));

    WriteStat stat{};
    bfc.setTransforms(vec, "me", false, &stat);

    iter_acc++;
  }
};


template <>
struct BufferCoreWrapper<SiloBufferCore>{
  SiloBufferCore bfc{};

  void init(){
    make_base_station(bfc);
  }
  void read(size_t start) const{
    vector<string> frames{};
    for(size_t i = start; i < start + FLAGS_read_len; i++){
      frames.push_back("link" + to_string(i));
    }
    bfc.justReadFrames(frames);
  }

  void write(size_t id, double nano_time, size_t &iter_acc){
    vector<TransformStamped> vec{};
    vec.push_back(trans("map", "link" + to_string(id), nano_time));

    WriteStat stat{};
    bfc.setTransforms(vec, "me", false, &stat);

    iter_acc++;
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
  double aborts; // should be zero expect latest

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
  CountAccum<chrono::duration<double>> latencies_acc_read_thread(read_threads);

  for(size_t t = 0; t < read_threads; t++){
    threads.emplace_back([t,&wait, &bfc_w,  &latencies_acc_read_thread,
                           &throughput_acc_read_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());
      while (wait){;}
      chrono::duration<double> latency_iter_acc{};
      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;

      size_t read_start = 0;

      for(;;){
        //        size_t start = r.next() % FLAGS_vehicle;

        auto before = chrono::steady_clock::now();
        bfc_w.read(read_start);
        auto after = chrono::steady_clock::now();

        latency_iter_acc += after - before;

        if(FLAGS_frequency != 0){
          this_thread::sleep_for(operator""s((1. / FLAGS_frequency)));
        }

        iter_count++;

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s(read_start+1)){
          read_start++;
        }

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      throughput_acc_read_thread.record(t, throughput(end_iter - start_iter, iter_count));
      latencies_acc_read_thread.record(t, latency_iter_acc / (double) iter_count);
    });
  }

  CountAccum<double> throughput_acc_write_thread(write_threads);
  CountAccum<chrono::duration<double>> latencies_acc_write_thread(write_threads);

  for(size_t t = 0; t < write_threads; t++){
    threads.emplace_back([t, &bfc_w, &wait,
                          &throughput_acc_write_thread,
                          &latencies_acc_write_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());
      while (wait){;}

      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;
      size_t iter_count = 0;
      chrono::duration<double> latency_iter_acc{};

      size_t write_offset = 0;

      for(;;){
        size_t id = r.next() % FLAGS_vehicle + write_offset;

        vector<TransformStamped> vec{};
        auto before = chrono::steady_clock::now();
        double nano = chrono::duration<double>(before.time_since_epoch()).count(); // from sec
        bfc_w.write(id, nano, iter_count);
        auto after = chrono::steady_clock::now();
        latency_iter_acc += after - before;

        if(FLAGS_frequency != 0){
          this_thread::sleep_for(operator""s((1. / FLAGS_frequency)));
        }

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s((double)(write_offset+1) * FLAGS_insert_span)){
          write_offset++;
          bfc_w.write(write_offset+FLAGS_vehicle, nano, iter_count);
        }

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      throughput_acc_write_thread.record(t, throughput(end_iter - start_iter, iter_count));
      latencies_acc_write_thread.record(t, latency_iter_acc / (double) iter_count);
    });
  }

  bfc_w.init();
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

  result.readThroughput = throughput_acc_read_thread.sum();
  result.writeThroughput = throughput_acc_write_thread.sum();


  return result;
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("car check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  CONSOLE_BRIDGE_logInform("thread: %d", FLAGS_thread);
  CONSOLE_BRIDGE_logInform("vehicle: %d", FLAGS_vehicle);
  CONSOLE_BRIDGE_logInform("read ratio: %lf", FLAGS_read_ratio);
  if(!(0. <= FLAGS_read_ratio and FLAGS_read_ratio <= 1.)){
    CONSOLE_BRIDGE_logError("wrong read ratio");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("read len: %d", FLAGS_read_len);
  if(!(0 <= FLAGS_read_len and FLAGS_read_len <= FLAGS_vehicle)){
    CONSOLE_BRIDGE_logError("wrong read len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("write len: %d", FLAGS_write_len);
  if(!(0 <= FLAGS_write_len and FLAGS_write_len <= FLAGS_vehicle)){
    CONSOLE_BRIDGE_logError("wrong write len");
    exit(-1);
  }
  CONSOLE_BRIDGE_logInform("Output: %s", FLAGS_output.c_str());
  CONSOLE_BRIDGE_logInform("frequency: %lf", FLAGS_frequency);
  CONSOLE_BRIDGE_logInform("loop sec: %d", FLAGS_loop_sec);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  cout << std::setprecision(std::numeric_limits<double>::digits10);

  RunResult old_result{};
  if(FLAGS_only == 0 or FLAGS_only == 1){
    BufferCoreWrapper<OldBufferCore> bfc_w{};
    old_result = run(bfc_w);
    cout << "old: " << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(old_result.time).count() << "ms" << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(old_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(old_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "throughput: " << old_result.throughput << endl;
    cout << "\t" << "read throughput: " << old_result.readThroughput << endl;
    cout << "\t" << "write throughput: " << old_result.writeThroughput << endl;
  }


  RunResult xact_result{};
  if(FLAGS_only == 0 or FLAGS_only == 2){
    BufferCoreWrapper<BufferCore> bfc_w_xact{};
    xact_result = run(bfc_w_xact);

    cout << "xact: " << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(xact_result.time).count() << "ms" << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(xact_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(xact_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "throughput: " << xact_result.throughput << endl;
    cout << "\t" << "read throughput: " << xact_result.readThroughput << endl;
    cout << "\t" << "write throughput: " << xact_result.writeThroughput << endl;
  }

  RunResult silo_result{};
  if(FLAGS_only == 0 or FLAGS_only == 3){
    BufferCoreWrapper<SiloBufferCore> bfc_w_silo{};
    silo_result = run(bfc_w_silo);

    cout << "silo: " << endl;
    cout << "\t" << "time: " << chrono::duration<double, std::milli>(silo_result.time).count() << "ms" << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(silo_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(silo_result.writeLatency).count() << "ms" << endl;
    cout << "\t" << "throughput: " << silo_result.throughput << endl;
    cout << "\t" << "read throughput: " << silo_result.readThroughput << endl;
    cout << "\t" << "write throughput: " << silo_result.writeThroughput << endl;
  }

  if(FLAGS_frequency != 0){
    cout << "\033[31mWarn: frequency defined, so throughput is not making any sense!\033[0m" << endl;
  }

  output << FLAGS_thread << " "; // 1
  output << FLAGS_vehicle << " "; // 2
  output << FLAGS_read_ratio << " "; // 3
  output << FLAGS_read_len << " "; // 4
  output << FLAGS_write_len << " "; // 5
  output << FLAGS_frequency << " "; // 6
  output << old_result.throughput << " "; // 7
  output << xact_result.throughput << " "; // 8
  output << chrono::duration<double, std::milli>(old_result.readLatency).count() << " "; // 9
  output << chrono::duration<double, std::milli>(xact_result.readLatency).count() << " "; // 10
  output << chrono::duration<double, std::milli>(old_result.writeLatency).count() << " "; // 11
  output << chrono::duration<double, std::milli>(xact_result.writeLatency).count() << " "; // 12
  output << endl;
  output.close();

  return 0;
}
