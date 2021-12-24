#include <tbb/concurrent_vector.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <set>
#include <cassert>
#include "../include/tf2/rwlock.h"

using namespace std;

constexpr size_t ITER = 1000000;
constexpr size_t JOINT = 100;
constexpr double WRITE_LEN = 0.5;

int main(int argc, char **argv){
  cout << "ITER: " << ITER << endl;
  mutex mutex_{};
  std::vector<RWLock> locks(JOINT+1);
  atomic_bool wait{true};

  vector<thread> threads{};
  for(size_t i = 0; i < 4; i++){
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

  for(size_t i = 0; i < 4; i++){
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
}
