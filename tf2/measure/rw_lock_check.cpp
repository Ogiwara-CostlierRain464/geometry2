#include <tbb/concurrent_vector.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include <iostream>
#include <set>
#include <cassert>
#include <gflags/gflags.h>
#include "../include/tf2/rwlock.h"

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread count");
DEFINE_uint64(iter, 100'000, "Iteration count");
DEFINE_string(output, "/tmp/a.dat", "Output file");

constexpr size_t JOINT = 100;
constexpr double WRITE_LEN = 0.5;

int main(int argc, char **argv){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  cout << "ITER: " << FLAGS_iter << endl;
  cout << "Output: " << FLAGS_output.c_str() << endl;
  mutex mutex_{};
  RWLock mutex2_{};
  std::vector<RWLock> locks(JOINT+1);
  std::vector<std::shared_ptr<RWLock>> locks2(JOINT+1, std::make_shared<RWLock>());
  atomic_bool wait{true};

  vector<thread> threads{};
  for(size_t i = 0; i < FLAGS_thread; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % JOINT;
        size_t until = link + (size_t)(JOINT * WRITE_LEN);
        if(until > JOINT) until = JOINT;
        for(size_t j = link; j < until; j++){
          mutex_.lock();
          mutex_.unlock();
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
  auto mutex_diff = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "mutex: " << mutex_diff.count() << endl;

  threads.clear();
  wait = true;

  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % JOINT;
        size_t until = link + (size_t) (JOINT * WRITE_LEN);
        if(until > JOINT) until = JOINT;
        for(size_t j = link; j < until; j++){
          mutex2_.w_lock();
          mutex2_.w_unlock();
        }
      }
    });
  }

  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }

  finish = chrono::high_resolution_clock::now();
  auto mutex2_diff = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "mutex2: " << mutex2_diff.count() << endl;

  return 0;

  threads.clear();
  wait = true;

  for(size_t i = 0; i < FLAGS_thread; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % JOINT;
        size_t until = link + (size_t) (JOINT * WRITE_LEN);
        if(until > JOINT) until = JOINT;
        for(size_t j = link; j < until; j++){
          locks[j].w_lock();
          locks[j].w_unlock();
        }
      }
    });
  }

  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }

  finish = chrono::high_resolution_clock::now();
  auto rw_diff = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "RW: " << rw_diff.count() << endl;

  threads.clear();
  wait = true;

  for(size_t i = 0; i < FLAGS_thread; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < FLAGS_iter; i++){
        size_t link = rand() % JOINT;
        size_t until = link + (size_t) (JOINT * WRITE_LEN);
        if(until > JOINT) until = JOINT;
        for(size_t j = link; j < until; j++){
          locks2[j]->w_lock();
          locks2[j]->w_unlock();
        }
      }
    });
  }

  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &e: threads){
    e.join();
  }

  finish = chrono::high_resolution_clock::now();
  auto rw_ptr_diff = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "RW ptr: " << rw_ptr_diff.count() << endl;

  output << FLAGS_iter << " ";
  output << mutex_diff.count() << " ";
  output << rw_diff.count() << " ";
  output << rw_ptr_diff.count() << endl;
  output.close();

  return 0;
}
