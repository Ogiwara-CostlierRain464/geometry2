#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread num");
DEFINE_uint64(iter, 10000, "iter");

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<size_t> results{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    results.emplace_back();
  }

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &results](){
      for(size_t i = 0; i < FLAGS_iter; i++){
        auto t1 = chrono::high_resolution_clock::now();
        auto t2 = chrono::high_resolution_clock::now();
        auto nano = chrono::duration_cast<chrono::nanoseconds>(t2 - t1);
        results[t] += nano.count();
      }
    });
  }

  for(auto &e: threads){
    e.join();
  }

  size_t acc{};
  for(auto &e: results){
    acc += e;
  }
  cout << "delay: " << acc / FLAGS_thread << endl;

  return 0;
}