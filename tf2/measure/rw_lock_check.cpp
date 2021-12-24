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

DEFINE_uint64(iter, 10'000, "Iteration count");
DEFINE_string(output, "/tmp/a.dat", "Output file");

size_t ITER = FLAGS_iter;
constexpr size_t JOINT = 100;
constexpr double WRITE_LEN = 0.5;
size_t THREADS = std::thread::hardware_concurrency();

int main(int argc, char **argv){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);

  cout << "ITER: " << ITER << endl;
  mutex mutex_{};
  std::vector<RWLock> locks(JOINT+1);
  std::vector<RWLockPtr> locks2(JOINT+1, std::make_shared<RWLock>());
  atomic_bool wait{true};

  vector<thread> threads{};
  for(size_t i = 0; i < THREADS; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
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
  auto micro = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "mutex: " << micro.count() << endl;

  threads.clear();
  wait = true;

  for(size_t i = 0; i < THREADS; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
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
  micro = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "RW: " << micro.count() << endl;

  threads.clear();
  wait = true;

  for(size_t i = 0; i < THREADS; i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
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
  micro = chrono::duration_cast<chrono::microseconds>(finish - start);
  cout << "RW ptr: " << micro.count() << endl;

  output << FLAGS_iter << " ";
  output << micro.count() << endl;
  output.close();

  return 0;
}
