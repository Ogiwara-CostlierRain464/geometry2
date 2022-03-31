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

#include <cassert>
#include <deque>
#include <sstream>
#include <ros/message_forward.h>
#include <ros/time.h>
#include <boost/shared_ptr.hpp>

namespace tf2{

typedef std::pair<ros::Time, tf2::CompactFrameID> P_TimeAndFrameID;

class TimeCache{
public:
  static const int MIN_INTERPOLATION_DISTANCE = 500; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 100000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 100ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache(bool is_static = false, ros::Duration max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));
  TimeCache(const TimeCache& other) = delete;
  TimeCache(TimeCache&& other) = delete;

  inline bool getData(ros::Time time,
                      tf2::TransformStorage & data_out,
                      std::string* error_str = nullptr){
    if(is_static){
      data_out = static_storage_;
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
  P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  unsigned int getListLength();
  inline ros::Time getLatestTimestamp(){
    if(is_static) return {};
    if (storage_.empty()) return ros::Time(); //empty list case
    return storage_.front().stamp_;
  }
  ros::Time getOldestTimestamp();


  typedef std::deque<tf2::TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;
  ros::Duration max_storage_time_;
  bool is_static;
  tf2::TransformStorage static_storage_{}; // for static

  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(tf2::TransformStorage*& one, tf2::TransformStorage*& two, ros::Time target_time, std::string* error_str);
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

    //Interpolate translation
    output.translation_.setInterpolate3(one.translation_, two.translation_, ratio);

    //Interpolate rotation
    output.rotation_ = slerp( one.rotation_, two.rotation_, ratio);

    output.stamp_ = time;
    output.frame_id_ = one.frame_id_;
    output.child_frame_id_ = one.child_frame_id_;
  }
  void pruneList();
};

}

#endif //GEOMETRY2_TIME_CACHE_H
