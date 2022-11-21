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

#include "tf2/stat.h"
#include "../old_tf2/old_buffer_core.h"
#include "../new_tf2/buffer_core.h"
#include "../include/tf2/xoroshiro128_plus.h"

using old_tf2::OldBufferCore;
using namespace geometry_msgs;
using namespace std;
using NewBuffer = new_tf2::BufferCore;
using std::chrono::operator""s;
using std::chrono::duration_cast;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "Thread size");
DEFINE_uint64(drone, 1'000, "Drone num");
DEFINE_string(output, "/tmp/f.dat", "Output file");
DEFINE_uint64(loop_sec, 10, "loop second");
DEFINE_string(only, "111", "Bit representation of enabled methods.");
#define READ_RATIO 0.5
#define WAIT_MILL_SEC 5

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

alignas(64) size_t obstacle_arr[1'000'000]{};

template <typename T>
void make_drone_group(T &bfc){
  auto now = chrono::steady_clock::now();
  double sec = chrono::duration<double>(now.time_since_epoch()).count();

  for(size_t i = 0; i < FLAGS_drone; i++){
    auto map_i = trans("map",
                       "drone" + to_string(i),
                       sec);
    bfc.setTransform(map_i, "me");
  }

  // insert obstacle from the beginning
  bfc.setTransform(trans("drone0", "0", sec), "me");
  for(size_t i = 0; i < FLAGS_drone; i++){
    obstacle_arr[i] = 0;
  }
}

template <typename T>
struct BufferCoreWrapper{
  void init(){}
  // read CTI drone_id->obstacle
  void read(size_t drone_id, size_t obstacle)const{}
  // update CTI drone_id->map at sec
  void update(size_t drone_id, double sec){}
  // insert CTI obstacle->drone_id
  void insert(size_t obstacle, size_t drone_id, double sec){}
};

template <>
struct BufferCoreWrapper<OldBufferCore>{
  OldBufferCore bfc{};
  void init(){
    bfc.clear();
    make_drone_group(bfc);
  }
  void read(size_t drone_id, size_t obstacle)const{
    bfc.lookupTransform("drone" + to_string(drone_id),
                        to_string(obstacle),
                        ros::Time(0));
  }

  void update(size_t drone_id, double sec){
    bfc.setTransform(trans("map", "drone" + to_string(drone_id), sec),
                     "me");
  }

  void insert(size_t obstacle, size_t drone_id, double sec){
    bfc.setTransform(trans("drone" + to_string(drone_id), to_string(obstacle), sec),
                     "me");
  }
};

enum NewAccess{
  TwoPL, Silo
};

template <>
struct BufferCoreWrapper<NewBuffer>{
  NewBuffer bfc;

  explicit BufferCoreWrapper(NewAccess access):
  bfc(access == Silo ? new_tf2::Silo : new_tf2::TwoPhaseLock){}

  void init(){
    make_drone_group(bfc);
  }
  void read(size_t drone_id, size_t obstacle)const{
    bfc.lookupLatestTransformXact("drone" + to_string(drone_id),
                                  to_string(obstacle));
  }

  void update(size_t drone_id, double sec){
    bfc.setTransform(trans("map", "drone" + to_string(drone_id), sec),
                     "me");
  }

  void insert(size_t obstacle, size_t drone_id, double sec){
    bfc.setTransform(trans("drone" + to_string(drone_id), to_string(obstacle), sec),
                     "me");
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
  chrono::duration<double> readLatency;
  chrono::duration<double> writeLatency;
};

double throughput(chrono::duration<double> time, size_t iter){
  return ((double) iter) * (1. / chrono::duration<double>(time).count());
}

template <typename T>
RunResult run(BufferCoreWrapper<T> &bfc_w){
  auto read_threads = (size_t)std::round((double)FLAGS_thread * READ_RATIO);
  size_t write_threads = FLAGS_thread - read_threads;

  atomic_bool wait{true};
  vector<thread> threads{};

  CountAccum<chrono::duration<double>> latencies_acc_read(FLAGS_thread);

  // at here, insert_and_update_thread

  for(size_t t = 0; t < read_threads; t++){ // update&read
    threads.emplace_back([t,&wait, &bfc_w, &latencies_acc_read](){
      while (wait){;}
      chrono::duration<double> latency_iter_acc{};
      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;

      for(;;){
        // drones / read_threads
        // get obstacle id by each slot
        size_t per_read_thread = FLAGS_drone / FLAGS_thread;
        auto before = chrono::steady_clock::now();
        for(size_t d = t * per_read_thread; d < (t+1) * per_read_thread; d++){
          size_t obs = obstacle_arr[d];

          auto now = chrono::steady_clock::now();
          double now_sec = chrono::duration<double>(now.time_since_epoch()).count(); // from sec

          bfc_w.update(d, now_sec);

          bfc_w.read(d, obs);
        }
        auto after = chrono::steady_clock::now();

        latency_iter_acc += (after - before) / per_read_thread;

        this_thread::sleep_for(operator""ms(WAIT_MILL_SEC));

        iter_count++;

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      latencies_acc_read.record(t, latency_iter_acc / (double) iter_count);
    });
  }

  CountAccum<chrono::duration<double>> latencies_acc_write(write_threads);

  for(size_t t = 0; t < write_threads; t++){ // insert&update
    threads.emplace_back([t, &bfc_w, &wait,
                           &latencies_acc_write, read_threads, write_threads](){
      Xoroshiro128Plus r(read_threads + t);
      while (wait){;}

      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;
      size_t iter_count = 0;
      chrono::duration<double> latency_iter_acc{};

      // need to make unique id per thread
      size_t obstacle_acc = t+1;

      for(;;){
        size_t id = r.next() % FLAGS_drone;

        auto now = chrono::steady_clock::now();
        double now_sec = chrono::duration<double>(now.time_since_epoch()).count(); // from sec
        // After setting CTI, update the obstacle arr.
        bfc_w.insert(obstacle_acc, id, now_sec);

        now = chrono::steady_clock::now();
        now_sec = chrono::duration<double>(now.time_since_epoch()).count(); // from sec
        bfc_w.update(id, now_sec); // fix
        auto after = chrono::steady_clock::now();
        latency_iter_acc += after - now;

        // before here, we need to set
        //      map
        //    :1,3    :1
        //  d1      d2
        //  :2
        //  o1
        // d2->map is obsolete
        // this will be a problem when tf uses interpolation

        // insert thread: insert(o->d) and update(d->map)
        // update&read thread: update and read
        //  map
        // :1   :1
        // d1     d2
        //
        //  map
        // :1,3   :1
        // d1     d2
        // :2
        // o1
        //
        //  map
        // :1   :1
        // d1     d2
        //

        for(size_t i = 0; i < FLAGS_drone; i++){
          obstacle_arr[i] = obstacle_acc;
        }

        this_thread::sleep_for(operator""ms(WAIT_MILL_SEC));

        obstacle_acc+=write_threads;

        iter_count++;

        end_iter = chrono::steady_clock::now();

        if(end_iter - start_iter > operator""s(FLAGS_loop_sec)){
          break;
        }
      }

      latencies_acc_write.record(t, latency_iter_acc / (double) iter_count);
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
  result.readLatency = latencies_acc_read.average();
  result.writeLatency = latencies_acc_write.average();

  return result;
}


int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  CONSOLE_BRIDGE_logInform("thread: %d", FLAGS_thread);
  CONSOLE_BRIDGE_logInform("drone: %d", FLAGS_drone);
  CONSOLE_BRIDGE_logInform("Output: %s", FLAGS_output.c_str());
  CONSOLE_BRIDGE_logInform("Only: %s", FLAGS_only.c_str());

  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  cout << std::setprecision(std::numeric_limits<double>::digits10);
  std::bitset<3> bs(FLAGS_only);

  RunResult old_result{};
  if(bs[0]){
    BufferCoreWrapper<OldBufferCore> bfc_w{};
    old_result = run(bfc_w);

    cout << "old: " << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(old_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(old_result.readLatency).count() << "ms" << endl;
  }

  RunResult _2pl_result{};
  if(bs[1]){
    BufferCoreWrapper<NewBuffer> bfc_w(NewAccess::TwoPL);
    _2pl_result = run(bfc_w);

    cout << "2PL: " << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(_2pl_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(_2pl_result.readLatency).count() << "ms" << endl;
  }

  RunResult silo_result{};
  if(bs[2]){
    BufferCoreWrapper<NewBuffer> bfc_w(NewAccess::Silo);
    silo_result = run(bfc_w);

    cout << "Silo: " << endl;
    cout << "\t" << "read latency: " << chrono::duration<double, std::milli>(silo_result.readLatency).count() << "ms" << endl;
    cout << "\t" << "write latency: " << chrono::duration<double, std::milli>(silo_result.readLatency).count() << "ms" << endl;
  }

  output << FLAGS_thread << " "; // 1
  output << FLAGS_drone << " "; // 2
  output << chrono::duration<double, std::milli>(old_result.readLatency).count() << " "; // 3
  output << chrono::duration<double, std::milli>(_2pl_result.readLatency).count() << " "; // 4
  output << chrono::duration<double, std::milli>(silo_result.readLatency).count() << " "; // 5

  output << chrono::duration<double, std::milli>(old_result.writeLatency).count() << " "; // 6
  output << chrono::duration<double, std::milli>(_2pl_result.writeLatency).count() << " "; // 7
  output << chrono::duration<double, std::milli>(silo_result.writeLatency).count() << " "; // 8

  output << endl;
  output.close();

  // Fast exit, no need to wait memory delete.
  exit(0);
}