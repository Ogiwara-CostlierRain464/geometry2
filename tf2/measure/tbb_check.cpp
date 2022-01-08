#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <tbb/concurrent_vector.h>

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread num");
DEFINE_uint64(iter, 10000, "iter");
DEFINE_string(output, "/tmp/c.dat", "output file");

double throughput(chrono::duration<double> time, size_t iter){
  return ((double) iter) * (1. / chrono::duration<double>(time).count());
}

int main(int argc, char* argv[]){
  gflags::SetUsageMessage("tbb check");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "thread: " << FLAGS_thread << endl;
  cout << "iter: " << FLAGS_iter << endl;
  cout << "Output: " << FLAGS_output << endl;

  tbb::concurrent_vector<size_t> vec(FLAGS_thread);
  bool wait=true;

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &vec](){
      for(size_t i = 0; i < FLAGS_iter; i++){
        vec[t]++;
      }
    });
  }

  auto start = chrono::high_resolution_clock::now();
  for(auto &e: threads){
    e.join();
  }
  auto finish = chrono::high_resolution_clock::now();
  auto sec = std::chrono::duration<double>(finish - start);
  auto opes = FLAGS_thread * FLAGS_iter;
  auto throughput_ = throughput(sec, opes);

  cout << throughput_ << "ope/sec" << endl;

  ofstream output{};
  output.open(FLAGS_output.c_str(), std::ios_base::app);
  output << FLAGS_thread << " ";
  output << FLAGS_iter << " ";
  output << throughput_ << endl;
  output.close();

  return 0;
}