#ifndef GEOMETRY2_FULL_JITTER_H
#define GEOMETRY2_FULL_JITTER_H

#include <thread>
#include <chrono>
#include <cmath>
#include "xoroshiro128_plus.h"

class FullJitter{
  double attempt = 0;
  Xoroshiro128Plus rand;

  explicit FullJitter(size_t seed):
  rand(seed){}

  void randomSleep(){
    double expo_sleep_time_in_us = std::min(1000. , pow(1 * 2, attempt));
    double randomized_sleep_time_in_us =
      rand.next_between(0, expo_sleep_time_in_us);

    std::this_thread::sleep_for(
      std::chrono::operator""us(randomized_sleep_time_in_us));

    attempt++;
  }
};

#endif //GEOMETRY2_FULL_JITTER_H
