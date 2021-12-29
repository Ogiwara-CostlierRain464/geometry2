#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread num");
DEFINE_uint64(iter, 10000, "iter");
DEFINE_string(output, "/tmp/b.dat", "output file");

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("speed check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "thread: " << FLAGS_thread << endl;
  cout << "Output: " << FLAGS_output << endl;
  cout << "is_steady: " << (chrono::steady_clock::is_steady ? "true" : "false") << endl;

  std::vector<size_t> results{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    results.emplace_back();
  }

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &results](){
      for(size_t i = 0; i < FLAGS_iter; i++){
        auto t1 = chrono::steady_clock::now();
        auto t2 = chrono::steady_clock::now();
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
  auto delay = acc / FLAGS_thread;
  cout << "delay: " << delay << endl;

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);
  output << FLAGS_thread << " ";
  output << delay << endl;
  output.close();

  return 0;
}