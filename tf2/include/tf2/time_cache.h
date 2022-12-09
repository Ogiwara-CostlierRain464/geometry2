/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef GEOMETRY2_TIME_CACHE_H
#define GEOMETRY2_TIME_CACHE_H

#include "transform_storage.h"
#include "exceptions.h"
#include "LinearMath/Transform.h"
#include "stat.h"

#include <cassert>
#include <deque>
#include <sstream>
#include <ros/message_forward.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <boost/shared_ptr.hpp>

#include <malloc.h>
#include <cstdint>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <tbb/concurrent_vector.h>
#include "cc_queue.h"

namespace geometry_msgs
{
  ROS_DECLARE_MESSAGE(TransformStamped);
}

namespace tf2{

typedef std::pair<ros::Time, tf2::CompactFrameID> P_TimeAndFrameID;

// https://gist.github.com/donny-dont/1471329#file-aligned_allocator-cpp-L51
template <typename T, std::size_t Alignment>
class aligned_allocator
{
public:

  // The following will be the same for virtually all allocators.
  typedef T * pointer;
  typedef const T * const_pointer;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T value_type;
  typedef std::size_t size_type;
  typedef ptrdiff_t difference_type;

  T * address(T& r) const
  {
    return &r;
  }

  const T * address(const T& s) const
  {
    return &s;
  }

  std::size_t max_size() const
  {
    // The following has been carefully written to be independent of
    // the definition of size_t and to avoid signed/unsigned warnings.
    return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T);
  }


  // The following must be the same for all allocators.
  template <typename U>
  struct rebind
  {
    typedef aligned_allocator<U, Alignment> other;
  } ;

  bool operator!=(const aligned_allocator& other) const
  {
    return !(*this == other);
  }

  void construct(T * const p, const T& t) const
  {
    void * const pv = static_cast<void *>(p);

    new (pv) T(t);
  }

  void destroy(T * const p) const
  {
    p->~T();
  }

  // Returns true if and only if storage allocated from *this
  // can be deallocated from other, and vice versa.
  // Always returns true for stateless allocators.
  bool operator==(const aligned_allocator& other) const
  {
    return true;
  }


  // Default constructor, copy constructor, rebinding constructor, and destructor.
  // Empty for stateless allocators.
  aligned_allocator() { }

  aligned_allocator(const aligned_allocator&) { }

  template <typename U> aligned_allocator(const aligned_allocator<U, Alignment>&) { }

  ~aligned_allocator() { }


  // The following will be different for each allocator.
  T * allocate(const std::size_t n) const
  {
    // The return value of allocate(0) is unspecified.
    // Mallocator returns NULL in order to avoid depending
    // on malloc(0)'s implementation-defined behavior
    // (the implementation can define malloc(0) to return NULL,
    // in which case the bad_alloc check below would fire).
    // All allocators can return NULL in this case.
    if (n == 0) {
      return NULL;
    }

    // All allocators should contain an integer overflow check.
    // The Standardization Committee recommends that std::length_error
    // be thrown in the case of integer overflow.
    if (n > max_size())
    {
      throw std::length_error("aligned_allocator<T>::allocate() - Integer overflow.");
    }

    // Mallocator wraps malloc().
    void * const pv = aligned_alloc(Alignment, n * sizeof(T));

    // Allocators should throw std::bad_alloc in the case of memory allocation failure.
    if (pv == NULL)
    {
      throw std::bad_alloc();
    }

    return static_cast<T *>(pv);
  }

  void deallocate(T * const p, const std::size_t n) const
  {
    free(p);
  }


  // The following will be the same for all allocators that ignore hints.
  template <typename U>
  T * allocate(const std::size_t n, const U * /* const hint */) const
  {
    return allocate(n);
  }


  // Allocators are not required to be assignable, so
  // all allocators should have a private unimplemented
  // assignment operator. Note that this will trigger the
  // off-by-default (enabled under /Wall) warning C4626
  // "assignment operator could not be generated because a
  // base class assignment operator is inaccessible" within
  // the STL headers, but that warning is useless.
private:
  aligned_allocator& operator=(const aligned_allocator&);
};

class TimeCache{
public:
  static const int MIN_INTERPOLATION_DISTANCE = 500; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 100000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 100ULL * 1'000'000'000LL;

  TimeCache(bool is_static = false, ros::Duration max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));
  TimeCache(const TimeCache& other) = delete;
  TimeCache(TimeCache&& other) = delete;

  uint8_t findClosest(tf2::TransformStorage*& one, tf2::TransformStorage*& two, ros::Time target_time, std::string* error_str);

  bool getData(ros::Time time,
                      tf2::TransformStorage & data_out,
                      std::string* error_str = nullptr,
                      ReadStat *stat = nullptr){
    if(stat){
      stat->dequeSize = storage_.size();
    }

    if(is_static){
      if(!storage_.empty()){
        data_out = storage_.front();
      }else{
        data_out = TransformStorage{};
      }
      data_out.stamp_ = time;
      return true;
    }

    tf2::TransformStorage* p_temp_1;
    tf2::TransformStorage* p_temp_2;

    int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
    if (num_nodes == 0)
    {
      return false;
    }
    else if (num_nodes == 1)
    {
      data_out = *p_temp_1;
    }
    else if (num_nodes == 2)
    {
      if( p_temp_1->frame_id_ == p_temp_2->frame_id_)
      {
        interpolate(*p_temp_1, *p_temp_2, time, data_out);
      }
      else
      {
        data_out = *p_temp_1;
      }
    }
    else
    {
      assert(0);
    }

    return true;
  }

  bool insertData(const tf2::TransformStorage& new_data);
  void clearList();
  tf2::CompactFrameID getParent(ros::Time time, std::string* error_str);
  inline P_TimeAndFrameID getLatestTimeAndParent()
  {
    if(is_static){
      CompactFrameID id = storage_.empty() ? 0 : storage_.front().frame_id_;
      return std::make_pair(ros::Time(), id);
    }

    if (storage_.empty())
    {
      return std::make_pair(ros::Time(), 0);
    }

    const tf2::TransformStorage& ts = storage_.first();
    return std::make_pair(ts.stamp_, ts.frame_id_);
  }

  /// Debugging information methods
  unsigned int getListLength();
  inline ros::Time getLatestTimestamp(){
    if(is_static) return {};
    if (storage_.empty()) return ros::Time(); //empty list case
    return storage_.front().stamp_;
  }
  ros::Time getOldestTimestamp();

  CCQueue<50> storage_;
  ros::Duration max_storage_time_;
  bool is_static;

  /// A helper function for getData
  //Assumes storage is already locked for it
  inline void interpolate(const tf2::TransformStorage& one,
                          const tf2::TransformStorage& two,
                          ros::Time time, tf2::TransformStorage& output){
    // Check for zero distance case
    if( two.stamp_ == one.stamp_ )
    {
      output = two;
      return;
    }
    //Calculate the ratio
    tf2Scalar ratio = (time - one.stamp_).toSec() / (two.stamp_ - one.stamp_).toSec();

    Vector3 tmp;
    Vector3 one_vec(one.vec[0], one.vec[1], one.vec[2]);
    Vector3 two_vec(two.vec[0], two.vec[1], two.vec[2]);

    tmp.setInterpolate3(one_vec, two_vec, ratio);
    //Interpolate translation
    output.vec[0] = tmp[0];
    output.vec[1] = tmp[1];
    output.vec[2] = tmp[2];


    //Interpolate rotation
    output.rotation_ = slerp( one.rotation_, two.rotation_, ratio);

    output.stamp_ = time;
    output.frame_id_ = one.frame_id_;
    output.child_frame_id_ = one.child_frame_id_;
  }
};

}

#endif //GEOMETRY2_TIME_CACHE_H
