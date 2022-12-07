#include <chrono>
#include <gflags/gflags.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <random>
#include <atomic>
#include "../include/tf2/time_cache.h"
#include "../include/tf2/buffer_core.h"
#include "../old_tf2/time_cache.h"
#include "../include/tf2/xoroshiro128_plus.h"

using namespace std;

DEFINE_uint64(thread, std::thread::hardware_concurrency(), "thread num");
DEFINE_string(output, "/tmp/d.dat", "output file");
DEFINE_uint64(loop_sec, 10, "loop second");

double throughput(chrono::duration<double> time, size_t iter){
  return ((double) iter) * (1. / chrono::duration<double>(time).count());
}

template <typename T>
struct ArrWrapper{
  ros::Time read(size_t i) const{ assert(false); };
};

template <>
struct ArrWrapper<old_tf2::TimeCacheInterface*>{
  old_tf2::TimeCacheInterface** arr;
  explicit ArrWrapper(old_tf2::TimeCacheInterface **arr)
  : arr(arr){}

  auto read(size_t i) const{
    auto ptr = arr[i];
    return ptr->getLatestTimestamp();
  }
};

template <>
struct ArrWrapper<tf2::TimeCache*>{
  tf2::TimeCache** arr;
  explicit ArrWrapper(tf2::TimeCache **arr)
    : arr(arr){}

  auto read(size_t i) const{
    return arr[i]->getLatestTimestamp();
  }
};

template <>
struct ArrWrapper<tf2::TransformStorage>{
  tf2::TransformStorage* arr;
  explicit ArrWrapper(tf2::TransformStorage *arr)
    : arr(arr){}

  auto read(size_t i) const{
    return arr[i].stamp_;
  }
};

template <>
struct ArrWrapper<std::deque<tf2::TransformStorage>>{
  std::deque<tf2::TransformStorage>* arr;
  explicit ArrWrapper(std::deque<tf2::TransformStorage> *arr)
    : arr(arr){}

  auto read(size_t i) const{
    return arr[i].front().stamp_;
  }
};

template <>
struct ArrWrapper<tf2::TimeCache>{
  tf2::TimeCache* arr;
  bool call_method;
  explicit ArrWrapper(tf2::TimeCache *arr, bool call_method = false)
    : arr(arr), call_method(call_method){}

  ros::Time read(size_t i) const{
    tf2::TransformStorage st;
    if(call_method){
      arr[i].getData(ros::Time(), st);
      return {};
    }else{
      auto e = &arr[i];
      if(e->is_static){
        st = e->storage_.front();
        st.stamp_ = {};
        return {};
      }

      if(ros::Time().isZero()){
        st = e->storage_.front();
        return {};
      }

      tf2::TransformStorage* p_temp_1;
      tf2::TransformStorage* p_temp_2;

      int num_nodes = e->findClosest(p_temp_1, p_temp_2, ros::Time(), nullptr);
      if (num_nodes == 0)
      {
        return {};
      }
      else if (num_nodes == 1)
      {
        st = *p_temp_1;
      }
      else if (num_nodes == 2)
      {
        if( p_temp_1->frame_id_ == p_temp_2->frame_id_)
        {
          e->interpolate(*p_temp_1, *p_temp_2, ros::Time(), st);
        }
        else
        {
          st = *p_temp_1;
        }
      }
      else
      {
        assert(0);
      }
    }
  }
};

struct A{
  std::deque<tf2::TransformStorage> storage_;
  ros::Duration max_storage_time_;
  bool is_static;

  ros::Time getLatestTimeStamp(){
    if(is_static) return {};
    if (storage_.empty()) return ros::Time(); //empty list case
    return storage_.front().stamp_;
  }
};

template <>
struct ArrWrapper<A>{
  A* arr;
  explicit ArrWrapper(A *arr)
    : arr(arr){}

  ros::Time read(size_t i) const{
    return arr[i].getLatestTimeStamp();
  }
};

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
double a(const ArrWrapper<T> &wrapper){
  CountAccum<double> throughput_acc_thread(FLAGS_thread);

  vector<thread> threads{};
  for(size_t t = 0; t < FLAGS_thread; t++){
    threads.emplace_back([t, &wrapper, &throughput_acc_thread](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());

      size_t iter_count = 0;
      auto start_iter = chrono::steady_clock::now();
      auto end_iter = start_iter;

      for(;;){
        auto time = wrapper.read(r.next() % 1'000'000);
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

  static_assert(sizeof(tf2::Quaternion) == 32, "");
  static_assert(sizeof(ros::Time) == 8, "");
  static_assert(sizeof(tf2::CompactFrameID) == 4, "");
  static_assert(sizeof(double [3]) == 24, "");
  cout << "vec: " << offsetof(tf2::TransformStorage, vec) << endl;
  cout << "rotation_: " << offsetof(tf2::TransformStorage, rotation_) << endl;
  cout << "frame_id_: " << offsetof(tf2::TransformStorage, frame_id_) << endl;
//  cout << sizeof(tf2::TransformStorage) << endl;
  static_assert(alignof(tf2::TransformStorage) == 128, "");
  static_assert(sizeof (tf2::TransformStorage) == 128, "");
  // 176
  //static_assert(sizeof(tf2::TimeCache) == sizeof(A), "");


  cout << "thread: " << FLAGS_thread << endl;
  cout << "Output: " << FLAGS_output << endl;
  cout << "Loop sec: " << FLAGS_loop_sec << endl;

  // double ref, dynamic dispatch
//  old_tf2::TimeCacheInterface **arr;
//  arr = new old_tf2::TimeCacheInterface*[1'000'000]();
//  for(size_t i = 0; i < 1'000'000; i++){
//    arr[i] = new old_tf2::TimeCache();
//    arr[i]->insertData({});
//  }

  // one ref
  tf2::TransformStorage *arr2;
  arr2 = new tf2::TransformStorage[1'000'000]();

  // one ref with deque wrap
//  std::deque<tf2::TransformStorage> *arr3;
//  arr3 = new std::deque<tf2::TransformStorage>[1'000'000]();
//  for(size_t i = 0; i < 1'000'000; i++){
//    arr3[i].emplace_back();
//  }

  // one ref with TimeCache wrap
  tf2::TimeCache *arr4;
  arr4 = new tf2::TimeCache[1'000'000]();
  for(size_t i = 0; i < 1'000'000; i++){
    arr4[i].insertData({});
  }

  // double ref, no dynamic dispatch
  tf2::TimeCache **arr5;
  arr5 = new tf2::TimeCache*[1'000'000]();
  for(size_t i = 0; i < 1'000'000; i++){
    arr5[i] = new tf2::TimeCache();
    arr5[i]->insertData({});
  }

  // custom wrap
//  A *arr6;
//  arr6 = new A[1'000'000]();
//  for(size_t i = 0; i < 1'000'000; i++){
//    arr6[i].storage_.emplace_back();
//  }


//  cout << "double ref, dynamic dispatch" << endl;
//  auto time_t = a(ArrWrapper<old_tf2::TimeCacheInterface*>(arr));
  cout << "one ref" << endl;
  auto storage_t = a(ArrWrapper<tf2::TransformStorage>(arr2));
//  cout << "one ref with deque wrap" << endl;
//  auto deque_t = a(ArrWrapper<std::deque<tf2::TransformStorage>>(arr3));
  cout << "one ref with TimeCache wrap: no method" << endl;
  auto time_cache_t = a(ArrWrapper<tf2::TimeCache>(arr4));
  cout << "one ref with TimeCache wrap: call method" << endl;
  auto time_cache_t2 = a(ArrWrapper<tf2::TimeCache>(arr4, true));

  cout << "double ref, no dynamic dispatch" << endl;
  auto time_t2 = a(ArrWrapper<tf2::TimeCache*>(arr5));
//  cout << "custom wrap" << endl;
//  auto a_t = a(ArrWrapper<A>(arr6));


//  ofstream output{};
//  output.open(FLAGS_output.c_str(), std::ios_base::app);
//  output << FLAGS_thread << " ";
//  output << time_t << " ";
//  output << storage_t << " ";
//  output << deque_t << " ";
//  output << endl;
//  output.close();

  return 0;
}