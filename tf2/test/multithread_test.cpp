#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <tf2/exceptions.h>

#include "../include/tf2/buffer_core.h"

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


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
