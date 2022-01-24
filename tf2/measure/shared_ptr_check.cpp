#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <random>
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
void a(T &arr){
  CountAccum<double> throughput_acc_thread(FLAGS_thread);

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &arr, &throughput_acc_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());

      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;
      size_t sum{};

      for(;;){
        sum += *arr[r.next() % 1'000'000];

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
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("shared ptr check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "thread: " << FLAGS_thread << endl;
  cout << "Output: " << FLAGS_output << endl;
  cout << "Loop sec: " << FLAGS_loop_sec << endl;

  vector<std::size_t*> raw(1'000'000, new size_t(1));
  vector<shared_ptr<size_t>> shared(1'000'000, make_shared<std::size_t>(1));

  a(raw);
  a(shared);

  return 0;
}