#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <random>
#include <atomic>
#include "xoroshiro128_plus.h"

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread num");
DEFINE_string(output, "/tmp/c.dat", "output file");
DEFINE_uint64(loop_sec, 60, "loop second");

double throughput(chrono::duration<double> time, size_t iter){
  return ((double) iter) * (1. / chrono::duration<double>(time).count());
}

template <typename T>
struct CountAccum{
  explicit CountAccum(size_t count): vec(count, T{}){}

  void record(size_t t_id, const T &data){
    vec[t_id] = data;
  };

  T sum(){
    return std::accumulate(vec.begin(), vec.end(), T{});
  }

  std::vector<T> vec;
};

template <typename T>
double a(T &arr){
  CountAccum<double> throughput_acc_thread(FLAGS_thread);

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &arr, &throughput_acc_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());

      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;

      for(;;){
        arr[r.next() % 1'000'000]->fetch_add(1);

        iter_count++;
        end_iter = chrono::steady_clock::now();
        if(end_iter - start_iter >  operator""s(FLAGS_loop_sec)){
          break;
        }
      }
      throughput_acc_thread.record(t, throughput(end_iter - start_iter, iter_count));
    });
  }

  auto start = chrono::high_resolution_clock::now();
  for(auto &e: threads){
    e.join();
  }
  auto finish = chrono::high_resolution_clock::now();
  auto sec = std::chrono::duration<double>(finish - start);
  auto throughput_ = throughput_acc_thread.sum();

  cout << throughput_ << "ope/sec" << endl;
  return throughput_;
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("shared ptr check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "thread: " << FLAGS_thread << endl;
  cout << "Output: " << FLAGS_output << endl;
  cout << "Loop sec: " << FLAGS_loop_sec << endl;

  vector<atomic_uint64_t *> raw(1'000'000, new atomic_uint64_t (1));
  vector<shared_ptr<atomic_uint64_t>> shared(1'000'000, make_shared<atomic_uint64_t>(1));

  auto raw_t = a(raw);
  auto shared_t = a(shared);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);
  output << FLAGS_thread << " ";
  output << shared_t << " ";
  output << raw_t << " ";
  output << endl;
  output.close();

  return 0;
}