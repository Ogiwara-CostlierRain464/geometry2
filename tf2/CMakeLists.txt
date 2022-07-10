cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -march=native")

project(tf2)

find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs rostime tf2_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(gflags REQUIRED)
add_subdirectory(third_party/tbb)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tf2
   DEPENDS console_bridge
   CATKIN_DEPENDS geometry_msgs tf2_msgs rostime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

file(GLOB_RECURSE TF2_SOURCES src/*.cpp)

add_library(tf2 ${TF2_SOURCES})
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} gflags tbb)
add_dependencies(tf2 ${catkin_EXPORTED_TARGETS})

install(TARGETS tf2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

file(GLOB_RECURSE SRC src/*.cpp)
file(GLOB_RECURSE INCLUDE include/*.h)

file(GLOB_RECURSE OLD_SRC old_tf2/*.cpp)
file(GLOB_RECURSE OLD_INCLUDE old_tf2/*.h)

add_executable(speed_check
  ${SRC}
  ${INCLUDE}
  ${OLD_SRC}
  ${OLD_INCLUDE}
  measure/speed_check.cpp
)
target_link_libraries(speed_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  gflags
  tbb
)
add_dependencies(speed_check ${catkin_EXPORTED_TARGETS})

add_executable(many_snakes
  ${SRC}
  ${INCLUDE}
  ${OLD_SRC}
  ${OLD_INCLUDE}
  measure/many_snakes.cpp
  )
target_link_libraries(many_snakes
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(many_snakes ${catkin_EXPORTED_TARGETS})

add_executable(car_check
  ${SRC}
  ${INCLUDE}
  ${OLD_SRC}
  ${OLD_INCLUDE}
  measure/car_check.cpp
  )
target_link_libraries(car_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(car_check ${catkin_EXPORTED_TARGETS})

add_executable(rw_lock_check
  ${SRC}
  ${INCLUDE}
  measure/rw_lock_check.cpp
  )
target_link_libraries(rw_lock_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(rw_lock_check ${catkin_EXPORTED_TARGETS})

add_executable(clock_check
  measure/clock_check.cpp
  )
target_link_libraries(clock_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gflags
  tbb
)
add_dependencies(clock_check ${catkin_EXPORTED_TARGETS})

add_executable(tbb_check
  measure/tbb_check.cpp
  )
target_link_libraries(tbb_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(tbb_check ${catkin_EXPORTED_TARGETS})

add_executable(shared_ptr_check
  measure/shared_ptr_check.cpp
  )
target_link_libraries(shared_ptr_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(shared_ptr_check ${catkin_EXPORTED_TARGETS})


add_executable(deque_check
  ${SRC}
  ${INCLUDE}
  ${OLD_SRC}
  ${OLD_INCLUDE}
  measure/deque_check.cpp
)
target_link_libraries(deque_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  gflags
  tbb
)
add_dependencies(deque_check
  ${catkin_EXPORTED_TARGETS}
)


add_executable(wait_check
  measure/wait_check.cpp
  )
target_link_libraries(wait_check
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gflags
  tbb
  )
add_dependencies(wait_check ${catkin_EXPORTED_TARGETS})


# Tests
if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(test_cache_unittest
  test/cache_unittest.cpp
  ${SRC}
  ${INCLUDE}
)
target_link_libraries(test_cache_unittest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(test_cache_unittest
  ${catkin_EXPORTED_TARGETS}
)

catkin_add_gtest(test_static_cache_unittest
  test/static_cache_test.cpp
  ${SRC}
  ${INCLUDE}
)
target_link_libraries(test_static_cache_unittest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(test_static_cache_unittest ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_simple
  test/simple_tf2_core.cpp
  ${SRC}
  ${INCLUDE}
)
target_link_libraries(test_simple
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

add_executable(speed_test
  EXCLUDE_FROM_ALL test/speed_test.cpp
  ${SRC}
  ${INCLUDE}
)

target_link_libraries(speed_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(tests speed_test)
add_dependencies(tests ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_rw_lock
  test/rw_lock_test.cpp
  include/tf2/rwlock.h
)
target_link_libraries(test_rw_lock
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(test_rw_lock ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_multithread
  test/multithread_test.cpp
  ${SRC}
  ${INCLUDE}
)
target_link_libraries(test_multithread
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${console_bridge_LIBRARIES}
  tbb
)
add_dependencies(test_multithread ${catkin_EXPORTED_TARGETS})


endif()
