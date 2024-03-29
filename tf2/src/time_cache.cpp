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

#include "../include/tf2/time_cache.h"
#include "../include/tf2/exceptions.h"
#include "../include/tf2/LinearMath/Vector3.h"
#include "../include/tf2/LinearMath/Quaternion.h"
#include "../include/tf2/LinearMath/Transform.h"
#include "../include/tf2/transform_storage.h"

#include <cassert>

namespace tf2{

TimeCache::TimeCache(bool is_static, ros::Duration max_storage_time)
  : max_storage_time_(max_storage_time), is_static(is_static)
{}

namespace cache { // Avoid ODR collisions https://github.com/ros/geometry2/issues/175
// hoisting these into separate functions causes an ~8% speedup.  Removing calling them altogether adds another ~10%
  void createExtrapolationException1(ros::Time t0, ros::Time t1, std::string* error_str)
  {
    if (error_str)
    {
      std::stringstream ss;
      ss << "Lookup would require extrapolation at time " << t0 << ", but only time " << t1 << " is in the buffer";
      *error_str = ss.str();
    }
  }

  void createExtrapolationException2(ros::Time t0, ros::Time t1, std::string* error_str)
  {
    if (error_str)
    {
      std::stringstream ss;
      ss << "Lookup would require extrapolation into the future.  Requested time " << t0 << " but the latest data is at time " << t1;
      *error_str = ss.str();
    }
  }

  void createExtrapolationException3(ros::Time t0, ros::Time t1, std::string* error_str)
  {
    if (error_str)
    {
      std::stringstream ss;
      ss << "Lookup would require extrapolation into the past.  Requested time " << t0 << " but the earliest data is at time " << t1;
      *error_str = ss.str();
    }
  }
} // namespace cache

uint8_t TimeCache::findClosest(tf2::TransformStorage*& one, tf2::TransformStorage*& two, ros::Time target_time, std::string* error_str)
{
  //No values stored
  if (storage_.empty())
  {
    return 0;
  }

  //If time == 0 return the latest
  if (target_time.isZero())
  {
    one = &storage_.latest();
    return 1;
  }

  // One value stored
  if (storage_.size() == 1)
  {
    tf2::TransformStorage& ts = storage_.latest();
    if (ts.stamp_ == target_time)
    {
      one = &ts;
      return 1;
    }
    else
    {
      cache::createExtrapolationException1(target_time, ts.stamp_, error_str);
      return 0;
    }
  }

  ros::Time latest_time = storage_.latest().stamp_;
  ros::Time earliest_time = storage_.first().stamp_;

  if (target_time == latest_time)
  {
    one = &storage_.latest();
    return 1;
  }
  else if (target_time == earliest_time)
  {
    one = &storage_.first();
    return 1;
  }
    // Catch cases that would require extrapolation
  else if (target_time > latest_time)
  {
    cache::createExtrapolationException2(target_time, latest_time, error_str);
    return 0;
  }
  else if (target_time < earliest_time)
  {
    cache::createExtrapolationException3(target_time, earliest_time, error_str);
    return 0;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  storage_.findTwoClose(target_time, one, two);
  return 2;
}

tf2::CompactFrameID TimeCache::getParent(ros::Time time, std::string* error_str)
{
  if(is_static){
    if(storage_.empty()){
      return 0;
    }else{
      return storage_.first().frame_id_;
    }
  }

  tf2::TransformStorage* p_temp_1;
  tf2::TransformStorage* p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0)
  {
    return 0;
  }

  return p_temp_1->frame_id_;
}

bool TimeCache::insertData(const tf2::TransformStorage& new_data)
{
  if(is_static){
    if(storage_.empty()){
      storage_.insert(new_data);
    }else{
      storage_.first() = new_data;
    }
    return true;
  }
  storage_.insert(new_data);

  return true;
}

void TimeCache::clearList()
{
  if(is_static)return;

  storage_.clear();
}

unsigned int TimeCache::getListLength()
{
  if(is_static)return 1;
  return storage_.size();
}

ros::Time TimeCache::getOldestTimestamp()
{
  if(is_static) return {};
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.first().stamp_;
}


}