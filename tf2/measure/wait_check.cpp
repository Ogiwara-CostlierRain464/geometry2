#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <random>
#include <atomic>
#include <cassert>
#include <mutex>
#include <set>
#include "../include/tf2/rwlock.h"

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

template<typename M>
struct MutexWrapper{
  void lock(){ assert(false); };
  void unlock(){}
};

template<>
struct MutexWrapper<std::mutex>{
  explicit MutexWrapper(std::mutex &mutex_)
  : mutex(mutex_){}

  void lock(){
    mutex.lock();
  };
  void unlock(){
    mutex.unlock();
  }

private:
  std::mutex &mutex;
};

template<>
struct MutexWrapper<RWLock>{
  explicit MutexWrapper(RWLock &mutex_)
    : mutex(mutex_){}

  void lock(){
    mutex.w_lock();
  };
  void unlock(){
    mutex.w_unlock();
  }

private:
  RWLock &mutex;
};



template <typename M>
double a(MutexWrapper<M> &mutex){
  CountAccum<double> throughput_acc_thread(FLAGS_thread);

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &mutex, &throughput_acc_thread](){
      std::random_device rnd;

      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;

      for(;;){
        mutex.lock();
        mutex.unlock();

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

  std::mutex m{};
  RWLock rw{};

  MutexWrapper<std::mutex> std(m);
  MutexWrapper<RWLock> me(rw);

  auto std_t = a(std);
  auto me_t = a(me);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);
  output << FLAGS_thread << " ";
  output << me_t << " ";
  output << std_t << " ";
  output << endl;
  output.close();

  return 0;
}