#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <tf2/exceptions.h>
#include <console_bridge/console.h>

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

TEST_F(MultithreadTest, setTransform_addTransformableCallback){
  BufferCore bfc;
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
  BufferCore bfc;
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
  BufferCore bfc;
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
  BufferCore bfc;
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
  BufferCore bfc;
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
  BufferCore bfc;
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
      bfc.setTransforms(vec, "me");
    }
  });

  auto start = chrono::high_resolution_clock::now();
  wait = false;
  t1.join(); t2.join();
  auto finish = chrono::high_resolution_clock::now();
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);
  return RUN_ALL_TESTS();
}
