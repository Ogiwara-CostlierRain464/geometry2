#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <tf2/exceptions.h>
#include <console_bridge/console.h>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_unordered_map.h>
#include <unordered_map>
#include <mutex>
#include <bitset>

#include "../include/tf2/buffer_core.h"
#include "../include/tf2/xoroshiro128_plus.h"
#include "../include/tf2/full_jitter.h"

using namespace tf2;
using namespace std;

struct MultithreadTest: public ::testing::Test{};

geometry_msgs::TransformStamped trans(
  const string &parent,
  const string &child,
  double sec){
  geometry_msgs::TransformStamped st;
  st.header.frame_id = parent;
  st.header.stamp = ros::Time(sec);
  st.child_frame_id = child;
  st.transform.rotation.w = 1;
  return st;
}

TEST_F(MultithreadTest, r_r_conf){
  BufferCore bfc;
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(1.5);

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 100'000; i++){
      bfc.lookupTransform("lidar", "map", when);
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 100'000; i++){
      bfc.lookupTransform("map", "lidar", when);
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, r_w_conf){
  BufferCore bfc(ros::Duration(10), 10005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.setTransform(trans("base_link", "lidar", (double)i), "me");
      bfc.setTransform(trans("map", "base_link", (double)i), "me");
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.lookupTransform("map", "lidar", when);
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, w_w_conf){
  BufferCore bfc;
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      double t = (double) i * 0.0001;
      bfc.setTransform(trans("base_link", "lidar", t), "me");
      bfc.setTransform(trans("map", "base_link", t), "me");
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      double t = (double) i * 0.0001 + 0.00005;
      bfc.setTransform(trans("base_link", "lidar", t), "not me");
      bfc.setTransform(trans("map", "base_link", t), "not me");
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, insert_read_conf){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      double t = (double) i * 0.001;
      bfc.setTransform(trans(to_string(i), to_string(i+1), t), "me");
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      try{
        bfc.lookupTransform(
          to_string(i), to_string(i+1), ros::Time(0));
      }catch(exception &e){;}
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}


TEST_F(MultithreadTest, egde_case_1){
  /**
   * A -> B  --|-------|--
   * B -> C          ----|------|--
   */
  BufferCore buffer{};
  buffer.setTransform(trans("B", "A", 1), "me");
  buffer.setTransform(trans("C", "B", 2), "me");

  EXPECT_THROW(buffer.lookupTransform("A", "C", ros::Time(0)), ExtrapolationException);
}

// You should write test including transform requests.
// where does transform requests comes from, actually?

TEST_F(MultithreadTest, setTransform_addTransformableCallback){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  auto handle = bfc.addTransformableCallback([&](
    TransformableRequestHandle req_handle,
    const string &,
    const string &,
    ros::Time,
    TransformableResult result
    ){
    ;
  });

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      double t = (double) i * 0.001;
      bfc.setTransform(trans(to_string(i), to_string(i+1), t), "me");
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "map", "lidar", ros::Time(0));
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, canTransform_addTransformableCallback){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  auto handle = bfc.addTransformableCallback([&](
    TransformableRequestHandle req_handle,
    const string &,
    const string &,
    ros::Time,
    TransformableResult result
  ){
    ;
  });

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.canTransform("lidar", "map", ros::Time(0), nullptr);
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "map", "lidar", ros::Time(0));
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, lookup_addTransformableCallback){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  auto handle = bfc.addTransformableCallback([&](
    TransformableRequestHandle req_handle,
    const string &,
    const string &,
    ros::Time,
    TransformableResult result
  ){
    ;
  });

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.lookupTransform("lidar", "map", ros::Time(0));
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "map", "lidar", ros::Time(0));
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, addTransformableCallback_addTransformableCallback){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  auto handle = bfc.addTransformableCallback([&](
    TransformableRequestHandle req_handle,
    const string &,
    const string &,
    ros::Time,
    TransformableResult result
  ){
    ;
  });

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "lidar", "map", ros::Time(0));
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "map", "lidar", ros::Time(0));
    }
  };
  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2);
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, setTransform_addTransformableCallback_addTransformableCallback){
  BufferCore bfc(ros::Duration(100), 10'005);
  // map <--> base_link <--> lidar
  bfc.setTransform(trans("map", "base_link", 1), "me");
  bfc.setTransform(trans("map", "base_link", 2), "me");
  bfc.setTransform(trans("base_link", "lidar", 1), "me");
  bfc.setTransform(trans("base_link", "lidar", 2), "me");
  ros::Time when(0);

  auto handle = bfc.addTransformableCallback([&](
    TransformableRequestHandle req_handle,
    const string &,
    const string &,
    ros::Time,
    TransformableResult result
  ){
    ;
  });

  atomic_bool wait{true};
  auto lam1 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "lidar", "map", ros::Time(0));
    }
  };
  auto lam2 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.addTransformableRequest(handle, "map", "lidar", ros::Time(0));
    }
  };
  auto lam3 = [&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.setTransform(trans("lidar", "map", (double)i * 0.0001), "me");
    }
  };

  auto start = chrono::high_resolution_clock::now();
  std::thread t1(lam1), t2(lam2), t3(lam3);
  wait = false;
  t1.join(); t2.join(); t3.join();
  auto finish = chrono::high_resolution_clock::now();
}


TEST_F(MultithreadTest, deadlock){
  BufferCore bfc(ros::Duration(100), 10'005);
  //   b
  //   |
  //   x
  //  / \
  // y   a
  // |
  // z
  bfc.setTransform(trans("x", "y", 0.00001), "me");
  bfc.setTransform(trans("y", "z", 0.00001), "me");
  bfc.setTransform(trans("x", "a", 0.00001), "me");
  bfc.setTransform(trans("b", "x", 0.00001), "me");

  atomic_bool wait{true};
  auto t1 = std::thread([&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      bfc.lookupTransform("a", "z", ros::Time(0));
    }
  });
  auto t2 = std::thread([&](){
    while (wait){;}
    for(size_t i = 0; i < 10'000; i++){
      std::vector<geometry_msgs::TransformStamped> vec{};
      vec.push_back(trans("x", "a", (double) i * 0.0001 ));
      vec.push_back(trans("b", "x", (double) i * 0.0001 ));
      bfc.setTransformsXact(vec, "me");
    }
  });

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}

TEST_F(MultithreadTest, concurrenct_vec){
  RWLock mutex{};
  vector<bool> vec{};
  tbb::concurrent_vector<bool> tbb_vec{};
  vector<thread> threads{};
  atomic_bool wait{true};
  size_t ITER = 1000'000;
  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        mutex.w_lock();
        vec.push_back(true);
        mutex.w_unlock();
      }
    });
  }
  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  auto finish = chrono::high_resolution_clock::now();
  cout << "vec: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;

  wait = true;
  threads.clear();
  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        tbb_vec.push_back(true);
      }
    });
  }
  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  finish = chrono::high_resolution_clock::now();
  cout << "tbb vec: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;
}

TEST_F(MultithreadTest, concurrenct_map){
  RWLock mutex{};
  unordered_map<int, string> map{};
  tbb::concurrent_unordered_map<int, string> tbb_map{};
  vector<thread> threads{};
  atomic_bool wait{true};
  size_t ITER = 100'000;
  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        mutex.w_lock();
        map.insert(make_pair(i, "aaaaa"));
        mutex.w_unlock();
      }
    });
  }
  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  auto finish = chrono::high_resolution_clock::now();
  cout << "map: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;

  wait = true;
  threads.clear();
  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        tbb_map.insert(make_pair(i, "aaaaa"));
      }
    });
  }
  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  finish = chrono::high_resolution_clock::now();
  cout << "tbb map: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;
}

TEST_F(MultithreadTest, rand_xor_cmp){
  size_t ITER = 100'000;

  vector<thread> threads{};
  atomic_bool wait{true};
  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        rand();
      }
    });
  }

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  auto finish = chrono::high_resolution_clock::now();
  cout << "rand: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;

  wait = true;
  threads.clear();

  for(size_t i = 0; i < std::thread::hardware_concurrency(); i++){
    threads.emplace_back([&](){
      std::random_device rnd;
      Xoroshiro128Plus r(rnd());
      while (wait){;}
      for(size_t i = 0; i < ITER; i++){
        r.next();
      }
    });
  }

  start = chrono::high_resolution_clock::now();
  wait = false;
  for(auto &t: threads){ t.join(); }
  finish = chrono::high_resolution_clock::now();
  cout << "xor: " << chrono::duration_cast<chrono::microseconds>(finish - start).count() << "us" << endl;
}

TEST_F(MultithreadTest, affinity){
  constexpr unsigned num_threads = 4;
  // A mutex ensures orderly access to std::cout from multiple threads.
  std::mutex iomutex;
  std::vector<std::thread> threads(num_threads);
  for (unsigned i = 0; i < num_threads; ++i) {
    threads[i] = std::thread([&iomutex, i] {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      for (size_t j = 0; j < 3; j++) {
        {
          // Use a lexical scope and lock_guard to safely lock the mutex only
          // for the duration of std::cout usage.
          std::lock_guard<std::mutex> iolock(iomutex);
          std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << "\n";
        }

        // Simulate important work done by the tread by sleeping for a bit...
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
      }
    });

    // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
    // only CPU i as set.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(i, &cpuset);
    int rc = pthread_setaffinity_np(threads[i].native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
      std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
    }
  }

  for (auto& t : threads) {
    t.join();
  }
}

TEST_F(MultithreadTest, vec_index){
  vector<std::size_t> vec{};
  vec.reserve(5);
  vec[0] = 9;
  vec[2] = 1;
  cout << "0: " << vec[0] << endl;
  cout << "2: " << vec[2] << endl;
}

TEST_F(MultithreadTest, fetch_add){
  atomic_uint64_t a{0};
  auto ret =  a.fetch_add(1);
  EXPECT_EQ(ret, 0);
}

TEST_F(MultithreadTest, bit_set){
  std::bitset<4> a("0");
  EXPECT_EQ(a[3], 0);
  std::bitset<4> b("0010");
  EXPECT_EQ(b[1], 1);
}

TEST_F(MultithreadTest, next_between){
  Xoroshiro128Plus x(0);
  EXPECT_TRUE(x.next_between(0,1000) <= 1000);
}

TEST_F(MultithreadTest, full_jitter){
  FullJitter j(0);
  //for(;;){ j.randomSleep(); }
}

#include "../include/tf2/cc_queue.h"

TEST_F(MultithreadTest, cc_queue){
  CCQueue<10> q;
  EXPECT_TRUE(q.empty());
  TransformStorage a{};
  a.stamp_.sec = 1;
  q.insert(a);
  a.stamp_.sec = 3;
  q.insert(a);
  a.stamp_.sec = 2;
  q.insert(a);
  EXPECT_EQ(q.size(), 3);
  ros::Time b{2.5};
  TransformStorage *one, *two;
  q.findTwoClose(b, one, two);
  EXPECT_EQ(one->stamp_.sec, 2);
  EXPECT_EQ(two->stamp_.sec, 3);
  auto latest = q.latest();
  EXPECT_EQ(latest.stamp_.sec, 3);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);
  return RUN_ALL_TESTS();
}
