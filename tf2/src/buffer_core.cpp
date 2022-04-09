/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "tf2/buffer_core.h"
#include "tf2/time_cache.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/TF2Error.h"

#include <assert.h>
#include <console_bridge/console.h>
#include "tf2/LinearMath/Transform.h"
#include <boost/foreach.hpp>

#include <thread>
#include <chrono>

using std::chrono::operator""ms;

namespace tf2
{

// Tolerance for acceptable quaternion normalization
  static double QUATERNION_NORMALIZATION_TOLERANCE = 10e-3;

/** \brief convert Transform msg to Transform */
  void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2)
  {tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

/** \brief convert Transform to Transform msg*/
  void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg)
  {
    msg.translation.x = tf2.getOrigin().x();
    msg.translation.y = tf2.getOrigin().y();
    msg.translation.z = tf2.getOrigin().z();
    msg.rotation.x = tf2.getRotation().x();
    msg.rotation.y = tf2.getRotation().y();
    msg.rotation.z = tf2.getRotation().z();
    msg.rotation.w = tf2.getRotation().w();
  }

/** \brief convert Transform to Transform msg*/
  void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
  {
    transformTF2ToMsg(tf2, msg.transform);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
  }

  void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::Transform& msg)
  {
    msg.translation.x = pos.x();
    msg.translation.y = pos.y();
    msg.translation.z = pos.z();
    msg.rotation.x = orient.x();
    msg.rotation.y = orient.y();
    msg.rotation.z = orient.z();
    msg.rotation.w = orient.w();
  }

  void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
  {
    transformTF2ToMsg(orient, pos, msg.transform);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
  }

  void setIdentity(geometry_msgs::Transform& tx)
  {
    tx.translation.x = 0;
    tx.translation.y = 0;
    tx.translation.z = 0;
    tx.rotation.x = 0;
    tx.rotation.y = 0;
    tx.rotation.z = 0;
    tx.rotation.w = 1;
  }

  bool startsWithSlash(const std::string& frame_id)
  {
    if (!frame_id.empty())
      if (frame_id[0] == '/')
        return true;
    return false;
  }

  std::string stripSlash(const std::string& in)
  {
    std::string out = in;
    if (startsWithSlash(in))
      out.erase(0,1);
    return out;
  }

  bool BufferCore::warnFrameId(const char* function_name_arg, const std::string& frame_id) const noexcept
  {
    if (frame_id.empty())
    {
      std::stringstream ss;
      ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
      CONSOLE_BRIDGE_logWarn("%s",ss.str().c_str());
      return true;
    }

    if (startsWithSlash(frame_id))
    {
      std::stringstream ss;
      ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
      CONSOLE_BRIDGE_logWarn("%s",ss.str().c_str());
      return true;
    }

    return false;
  }

  CompactFrameID BufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
  {
    if (frame_id.empty())
    {
      std::stringstream ss;
      ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
      throw tf2::InvalidArgumentException(ss.str());
    }

    if (startsWithSlash(frame_id))
    {
      std::stringstream ss;
      ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
      throw tf2::InvalidArgumentException(ss.str());
    }

    CompactFrameID id = lookupFrameNumber(frame_id);
    if (id == 0)
    {
      std::stringstream ss;
      ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
      throw tf2::LookupException(ss.str());
    }

    return id;
  }

  BufferCore::BufferCore(ros::Duration cache_time, uint64_t max_node_size, CCMethod cc)
    : cache_time_(cache_time)
    , transformable_callbacks_counter_(0)
    , transformable_requests_counter_(0)
    , using_dedicated_thread_(false)
    , cc(cc)
    , max_node_size_(max_node_size)
  {
    frames_ = new TimeCache[max_node_size]();
    if(cc == TwoPhaseLock){
      frame_rw_lock_ = new RWLock[max_node_size]();
    }else if(cc == Silo){
      frame_vrw_lock_ = new VRWLock[max_node_size]();
    }

    frameIDs_reverse = new std::string[max_node_size]();
    frame_authority_ = new std::string[max_node_size]();

    frameIDs_["NO_PARENT"] = 0;
    frameIDs_reverse[0] = "NO_PARENT";
  }

  void BufferCore::warmUpPages() noexcept{
    for(size_t i = 0; i < max_node_size_; i++){
      frames_[i].storage_.emplace_back();
      frames_[i].storage_.front().rotation_.m_floats[0] = 0.5;
      frames_[i].storage_.front().vec[0]= 0.5;
      frames_[i].storage_.pop_front();
      if(cc == TwoPhaseLock){
        frame_rw_lock_[i].w_lock();
        frame_rw_lock_[i].w_unlock();
      }else if(cc == Silo){
        frame_vrw_lock_[i].wLock();
        frame_vrw_lock_[i].wUnLock();
      }
      frameIDs_reverse[i] = "";
      frame_authority_[i] = "";
    }
  }

  BufferCore::~BufferCore()
  {
    delete[] frames_;

    if(cc == TwoPhaseLock){
      delete[] frame_rw_lock_;
    }else if(cc == Silo){
      delete[] frame_vrw_lock_;
    }
    delete[] frameIDs_reverse;
    delete[] frame_authority_;
  }

  void BufferCore::clear()
  {
    for(size_t i = 0; i < next_frame_id_; i++){
      frames_[i].clearList();
    }
  }

  bool BufferCore::setTransform(const geometry_msgs::TransformStamped& transform_in, const std::string& authority, bool is_static) noexcept
  {
    std::vector<geometry_msgs::TransformStamped> vec{transform_in};
    setTransformsXact(vec, authority, is_static);
  }

  inline geometry_msgs::TransformStamped stripTransform(const geometry_msgs::TransformStamped &tr){
    geometry_msgs::TransformStamped tmp = tr;
    tmp.header.frame_id = stripSlash(tmp.header.frame_id);
    tmp.child_frame_id = stripSlash(tmp.child_frame_id);
    return tmp;
  }

  inline bool checkTransformValid(const geometry_msgs::TransformStamped &tr, const std::string& authority){
    if (tr.child_frame_id == tr.header.frame_id)
    {
      CONSOLE_BRIDGE_logError("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), tr.child_frame_id.c_str());
      return false;
    }

    if (tr.child_frame_id.empty())
    {
      CONSOLE_BRIDGE_logError("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
      return false;
    }

    if (tr.header.frame_id.empty())
    {
      CONSOLE_BRIDGE_logError("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", tr.child_frame_id.c_str(), authority.c_str());
      return false;
    }

    if (std::isnan(tr.transform.translation.x)
        ||  std::isnan(tr.transform.translation.y)
        ||  std::isnan(tr.transform.translation.z)
        ||  std::isnan(tr.transform.rotation.x)
        ||  std::isnan(tr.transform.rotation.y)
        ||  std::isnan(tr.transform.rotation.z)
        ||  std::isnan(tr.transform.rotation.w))
    {
      CONSOLE_BRIDGE_logError("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
                              tr.child_frame_id.c_str(), authority.c_str(),
                              tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z,
                              tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w
      );
      return false;
    }

    bool valid = std::abs((tr.transform.rotation.w * tr.transform.rotation.w
                           + tr.transform.rotation.x * tr.transform.rotation.x
                           + tr.transform.rotation.y * tr.transform.rotation.y
                           + tr.transform.rotation.z * tr.transform.rotation.z) - 1.0f) < QUATERNION_NORMALIZATION_TOLERANCE;

    if (!valid){
      CONSOLE_BRIDGE_logError("TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of an invalid quaternion in the transform (%f %f %f %f)",
                              tr.child_frame_id.c_str(), authority.c_str(),
                              tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w);
      return false;
    }
    return true;
  }

  bool BufferCore::setTransformsXact(const std::vector<geometry_msgs::TransformStamped> &transforms,
                                     const std::string& authority, bool is_static, WriteStat *stat) noexcept
  {
    std::vector<geometry_msgs::TransformStamped> stripped{};
    stripped.reserve(transforms.size());
    for(auto &e: transforms){
      stripped.push_back(stripTransform(e));
    }


    for(auto &e: stripped){
      if(!checkTransformValid(e, authority)){
        return false;
      }
    }

    // before testTransformableRequests, you have to unlock.
    {
      std::vector<std::tuple<
        TimeCache*,
        geometry_msgs::TransformStamped,
        CompactFrameID>> write_set{};

      if(cc == TwoPhaseLock){
        ScopedWriteSetUnLocker un_locker(frame_rw_lock_);

        try_lock:
        for(auto &e: stripped){
          auto id = lookupOrInsertFrameNumber(e.child_frame_id);
          auto frame = getFrame(id);
          if(frame == nullptr){
            frame = allocateFrame(id, is_static);
          }

          if(un_locker.wLockedSize() == 0){
            while (!un_locker.tryWLockIfNot(id)){
              if(stat){
                stat->tryWriteCount++;
              }
            }
          }else{
            // wLockedSize() >= 1
            bool locked_suc = un_locker.tryWLockIfNot(id);
            if(!locked_suc){ // No-wait 2PL.
              if(stat != nullptr){
                stat->incAbort();
              }
              un_locker.unlockAll();
              write_set.clear();
              std::this_thread::sleep_for(1ms);
              goto try_lock;
            }
          }

          write_set.emplace_back(frame, e, id);
        }

        // all lock acquired, so write.
        for(auto &w: write_set){
          auto frame = std::get<0>(w);
          auto e = std::get<1>(w);
          auto id = std::get<2>(w);

          std::string err_str;
          if(frame->insertData(TransformStorage(e, lookupOrInsertFrameNumber(e.header.frame_id), id))){
            frame_authority_[id] = authority;
          }else{
            CONSOLE_BRIDGE_logWarn("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at https://wiki.ros.org/tf/Errors%%20explained", e.child_frame_id.c_str(), e.header.stamp.toSec(), authority.c_str());
            // TODO: Impl rollback
          }
        }
      }else if(cc == Silo){
        phase_1:
        for(auto &e: stripped){ // assert stripped is sorted.
          auto id = lookupOrInsertFrameNumber(e.child_frame_id);
          auto frame = getFrame(id);
          if(frame == nullptr){
            frame = allocateFrame(id, is_static);
          }

          frame_vrw_lock_[id].wLock();
          write_set.emplace_back(frame, e, id);
        }

        phase_2:
        ;// no read, so just skip
        phase_3:
        for(auto &w: write_set){
          auto frame = std::get<0>(w);
          auto e = std::get<1>(w);
          auto id = std::get<2>(w);

          std::string err_str;
          if(frame->insertData(TransformStorage(e, lookupOrInsertFrameNumber(e.header.frame_id), id))){
            frame_authority_[id] = authority;
          }else{
            CONSOLE_BRIDGE_logWarn("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at https://wiki.ros.org/tf/Errors%%20explained", e.child_frame_id.c_str(), e.header.stamp.toSec(), authority.c_str());
            // TODO: Impl rollback
          }

          frame_vrw_lock_[id].wUnLock();
        }
      }
    }
    // set Transform and test req aren't handled serialized fashion!
    testTransformableRequests();
    return true;
  }

  TimeCacheInterfacePtr BufferCore::allocateFrame(CompactFrameID cfid, bool is_static) noexcept
  {
    frames_[cfid].is_static = is_static;
    return &frames_[cfid];
  }

  enum WalkEnding
  {
    Identity,
    TargetParentOfSource,
    SourceParentOfTarget,
    FullPath,
  };

// TODO for Jade: Merge walkToTopParent functions; this is now a stub to preserve ABI
  template<typename F>
  int BufferCore::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const noexcept
  {
    return walkToTopParent(f, time, target_id, source_id, error_string, NULL);
  }

  template<typename F>
  int BufferCore::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id,
                                  CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID>
                                  *frame_chain, ReadStat *stat) const noexcept
  {
    if (frame_chain)
      frame_chain->clear();

    // Short circuit if zero length transform to allow lookups on non existant links
    if (source_id == target_id)
    {
      f.finalize(Identity, time);
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    //If getting the latest get the latest common time
    if (time == ros::Time())
    {
      int retval = getLatestCommonTime(target_id, source_id, time, error_string);
      if (retval != tf2_msgs::TF2Error::NO_ERROR)
      {
        return retval;
      }
    }

    // Walk the tree to its root from the source frame, accumulating the transform
    CompactFrameID frame = source_id;
    CompactFrameID top_parent = frame;
    uint32_t depth = 0;

    std::string extrapolation_error_string;
    bool extrapolation_might_have_occurred = false;

    while (frame != 0)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);
      if (frame_chain)
        frame_chain->push_back(frame);

      if (!cache)
      {
        // There will be no cache for the very root of the tree
        top_parent = frame;
        break;
      }

      CompactFrameID parent;
      if(cc == TwoPhaseLock){
        while (!frame_rw_lock_[frame].r_trylock()){
          if(stat){
            stat->tryReadLockCount++;
          }
        }
        parent = f.gather(cache, time, &extrapolation_error_string, stat);
        frame_rw_lock_[frame].r_unlock();
      }else if(cc == Silo) {
      retry:
        auto old_v = frame_vrw_lock_[frame].virtualRLock();
        parent = f.gather(cache, time, &extrapolation_error_string, stat);
        auto new_v = frame_vrw_lock_[frame].virtualRLock();
        if(old_v != new_v) goto retry;
      }

      if (parent == 0)
      {
        // Just break out here... there may still be a path from source -> target
        top_parent = frame;
        extrapolation_might_have_occurred = true;
        break;
      }

      // Early out... target frame is a direct parent of the source frame
      if (frame == target_id)
      {
        f.finalize(TargetParentOfSource, time);
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      f.accum(true);

      top_parent = frame;
      frame = parent;

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains a loop." << std::endl
             << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    // Now walk to the top parent from the target frame, accumulating its transform
    frame = target_id;
    depth = 0;
    std::vector<CompactFrameID> reverse_frame_chain;

    while (frame != top_parent)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);
      if (frame_chain)
        reverse_frame_chain.push_back(frame);

      if (!cache)
      {
        break;
      }

      CompactFrameID parent;
      if(cc == TwoPhaseLock){
        while (!frame_rw_lock_[frame].r_trylock()){
          if(stat){
            stat->tryReadLockCount++;
          }
        }
        parent = f.gather(cache, time, &extrapolation_error_string, stat);
        frame_rw_lock_[frame].r_unlock();
      }else if(cc == Silo) {
      retry2:
        auto old_v = frame_vrw_lock_[frame].virtualRLock();
        parent = f.gather(cache, time, &extrapolation_error_string, stat);
        auto new_v = frame_vrw_lock_[frame].virtualRLock();
        if(old_v != new_v) goto retry2;
      }

      if (parent == 0)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
          *error_string = ss.str();
        }

        return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      }

      // Early out... source frame is a direct parent of the target frame
      if (frame == source_id)
      {
        f.finalize(SourceParentOfTarget, time);
        if (frame_chain)
        {
          // Use the walk we just did
          frame_chain->swap(reverse_frame_chain);
          // Reverse it before returning because this is the reverse walk.
          std::reverse(frame_chain->begin(), frame_chain->end());
        }
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      f.accum(false);

      frame = parent;

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains a loop." << std::endl
             << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    if (frame != top_parent)
    {
      if (extrapolation_might_have_occurred)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << extrapolation_error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
          *error_string = ss.str();
        }

        return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;

      }

      createConnectivityErrorString(source_id, target_id, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }
    else if (frame_chain){
      // append top_parent to reverse_frame_chain for easier matching/trimming
      reverse_frame_chain.push_back(frame);
    }

    f.finalize(FullPath, time);
    if (frame_chain)
    {
      // Pruning: Compare the chains starting at the parent (end) until they differ
      int m = reverse_frame_chain.size()-1;
      int n = frame_chain->size()-1;
      for (; m >= 0 && n >= 0; --m, --n)
      {
        if ((*frame_chain)[n] != reverse_frame_chain[m])
        {
          break;
        }
      }
      // Erase all duplicate items from frame_chain
      if (n > 0)
      {
        // N is offset by 1 and leave the common parent for this result
        frame_chain->erase(frame_chain->begin() + (n + 2), frame_chain->end());
      }
      if (m < reverse_frame_chain.size())
      {
        for (int i = m; i >= 0; --i)
        {
          frame_chain->push_back(reverse_frame_chain[i]);
        }
      }
    }

    return tf2_msgs::TF2Error::NO_ERROR;
  }

  template<typename F>
  int BufferCore::walkToTopParentLatest(F& f, CompactFrameID target_id,
                                  CompactFrameID source_id, std::string* error_string,
                                  ReadStat *stat) const noexcept
  {
retry:
    ScopedWriteSetUnLocker un_locker(frame_rw_lock_);
    ReadChecker read_checker(frame_vrw_lock_);

    // Short circuit if zero length transform to allow lookups on non existant links
    if (source_id == target_id)
    {
      f.finalize(Identity, ros::Time(0));
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    // Walk the tree to its root from the source frame, accumulating the transform
    CompactFrameID frame = source_id;
    CompactFrameID top_parent = frame;
    uint32_t depth = 0;

    std::string extrapolation_error_string;
    bool extrapolation_might_have_occurred = false;

    while (frame != 0)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);
      if (!cache)
      {
        // There will be no cache for the very root of the tree
        top_parent = frame;
        break;
      }

      if(cc == TwoPhaseLock){
        un_locker.rLockIfNot(frame);
      } else if(cc == Silo){
        read_checker.addRLock(frame);
      }

      if(stat != nullptr){
        stat->timestamps.push_back(cache->getLatestTimestamp().toNSec());
      }

      CompactFrameID parent = f.gatherLatest(cache);

      if (parent == 0)
      {
        // Just break out here... there may still be a path from source -> target
        top_parent = frame;
        extrapolation_might_have_occurred = true;
        break;
      }

      // Early out... target frame is a direct parent of the source frame
      if (frame == target_id)
      {
        if(cc == Silo and !read_checker.check()){
          if(stat){
            stat->abortCount++;
            stat->timestamps.clear();
          }
          read_checker.clear();
          goto retry;
        }

        f.finalize(TargetParentOfSource, ros::Time(0));
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      f.accum(true);

      top_parent = frame;
      frame = parent;

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains a loop." << std::endl
             << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    // Now walk to the top parent from the target frame, accumulating its transform
    frame = target_id;
    depth = 0;

    while (frame != top_parent)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);
      if (!cache)
      {
        break;
      }

      if(cc == TwoPhaseLock){
        un_locker.rLockIfNot(frame);
      } else if(cc == Silo){
        read_checker.addRLock(frame);
      }

      if(stat != nullptr){
        stat->timestamps.push_back(cache->getLatestTimestamp().toNSec());
      }

      CompactFrameID parent = f.gatherLatest(cache);

      if (parent == 0)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
          *error_string = ss.str();
        }

        return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      }

      // Early out... source frame is a direct parent of the target frame
      if (frame == source_id)
      {
        if(cc == Silo and !read_checker.check()){
          if(stat){
            stat->abortCount++;
            stat->timestamps.clear();
          }
          read_checker.clear();
          goto retry;
        }

        f.finalize(SourceParentOfTarget, ros::Time(0));
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      f.accum(false);

      frame = parent;

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains a loop." << std::endl
             << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    if (frame != top_parent)
    {
      if (extrapolation_might_have_occurred)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss << extrapolation_error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      }
      createConnectivityErrorString(source_id, target_id, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    if(cc == Silo and !read_checker.check()){
      if(stat){
        stat->abortCount++;
        stat->timestamps.clear();
      }
      read_checker.clear();
      goto retry;
    }

    f.finalize(FullPath, ros::Time(0));

    return tf2_msgs::TF2Error::NO_ERROR;
  }

  struct TransformAccum
  {
    TransformAccum()
      : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
      , source_to_top_vec(0.0, 0.0, 0.0)
      , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
      , target_to_top_vec(0.0, 0.0, 0.0)
      , result_quat(0.0, 0.0, 0.0, 1.0)
      , result_vec(0.0, 0.0, 0.0)
      , st_rotation_(0.0, 0.0, 0.0, 1.0)
      , st_translation_(0.0, 0.0, 0.0)
    {
    }

    inline CompactFrameID gather(TimeCacheInterfacePtr cache, ros::Time time, std::string* error_string , ReadStat *stat)
    {
      TransformStorage st;
      if (!cache->getData(time, st, error_string, stat))
      {
        return 0;
      }

      // For performance reason, we avoid to copy whole TransformStorage struct, but only copy
      // required fields.
      st_rotation_.m_floats[0] = st.rotation_.m_floats[0];
      st_rotation_.m_floats[1] = st.rotation_.m_floats[1];
      st_rotation_.m_floats[2] = st.rotation_.m_floats[2];
      st_rotation_.m_floats[3] = st.rotation_.m_floats[3];

      st_translation_.m_floats[0] = st.vec[0];
      st_translation_.m_floats[1] = st.vec[1];
      st_translation_.m_floats[2] = st.vec[2];

      return st.frame_id_;
    }

    inline CompactFrameID gatherLatest(TimeCacheInterfacePtr cache)
    {
      if(cache->storage_.empty()){
        return 0;
      }
      // For performance reason, we avoid to copy whole TransformStorage struct, but only copy
      // required fields.
      auto st = cache->storage_.front();
      st_rotation_.m_floats[0] = st.rotation_.m_floats[0];
      st_rotation_.m_floats[1] = st.rotation_.m_floats[1];
      st_rotation_.m_floats[2] = st.rotation_.m_floats[2];
      st_rotation_.m_floats[3] = st.rotation_.m_floats[3];
      st_translation_.m_floats[0] = st.vec[0];
      st_translation_.m_floats[1] = st.vec[1];
      st_translation_.m_floats[2] = st.vec[2];
      return st.frame_id_;
    }


    inline void accum(bool source)
    {
      if (source)
      {
        source_to_top_vec = quatRotate(st_rotation_, source_to_top_vec) + st_translation_;
        source_to_top_quat = st_rotation_ * source_to_top_quat;
      }
      else
      {
        target_to_top_vec = quatRotate(st_rotation_, target_to_top_vec) + st_translation_;
        target_to_top_quat = st_rotation_ * target_to_top_quat;
      }
    }

    inline void finalize(WalkEnding end, ros::Time _time)
    {
      switch (end)
      {
        case Identity:
          break;
        case TargetParentOfSource:
          result_vec = source_to_top_vec;
          result_quat = source_to_top_quat;
          break;
        case SourceParentOfTarget:
        {
          tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
          tf2::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
          result_vec = inv_target_vec;
          result_quat = inv_target_quat;
          break;
        }
        case FullPath:
        {
          tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
          tf2::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

          result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
          result_quat = inv_target_quat * source_to_top_quat;
        }
          break;
      };

      time = _time;
    }

    tf2::Quaternion st_rotation_;
    tf2::Vector3 st_translation_;
    ros::Time time;
    tf2::Quaternion source_to_top_quat;
    tf2::Vector3 source_to_top_vec;
    tf2::Quaternion target_to_top_quat;
    tf2::Vector3 target_to_top_vec;

    tf2::Quaternion result_quat;
    tf2::Vector3 result_vec;
  };

  geometry_msgs::TransformStamped BufferCore::lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& time, ReadStat *stat) const noexcept(false)
  {
    if (target_frame == source_frame) {
      geometry_msgs::TransformStamped identity;
      identity.header.frame_id = target_frame;
      identity.child_frame_id = source_frame;
      identity.transform.rotation.w = 1;

      if (time == ros::Time())
      {
        CompactFrameID target_id = lookupFrameNumber(target_frame);
        TimeCacheInterfacePtr cache = getFrame(target_id);
        if (cache) {
          if(cc == TwoPhaseLock){
            frame_rw_lock_[target_id].r_lock();
            identity.header.stamp = cache->getLatestTimestamp();
            frame_rw_lock_[target_id].r_unlock();
          }else if(cc == Silo){
          retry:
            auto old_v = frame_vrw_lock_[target_id].virtualRLock();
            identity.header.stamp = cache->getLatestTimestamp();
            auto new_v = frame_vrw_lock_[target_id].virtualRLock();
            if(old_v != new_v) goto retry;
          }
        }else
          identity.header.stamp = time;
      }
      else
        identity.header.stamp = time;

      return identity;
    }

    //Identify case does not need to be validated above
    CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
    CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);
    assert(target_id != source_id);

    std::string error_string;
    TransformAccum accum;
    int retval = walkToTopParent(accum, time, target_id, source_id, &error_string, nullptr, stat);
    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
        case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw ConnectivityException(error_string);
        case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw ExtrapolationException(error_string);
        case tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw LookupException(error_string);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
          assert(false);
      }
    }

    geometry_msgs::TransformStamped output_transform;
    transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform, accum.time, target_frame, source_frame);
    return output_transform;
  }

  geometry_msgs::TransformStamped BufferCore::lookupLatestTransformXact(
    const std::string& target_frame,
    const std::string& source_frame,
    ReadStat *stat) const noexcept(false)
  {
    if (target_frame == source_frame) {
      geometry_msgs::TransformStamped identity;
      identity.header.frame_id = target_frame;
      identity.child_frame_id = source_frame;
      identity.transform.rotation.w = 1;

      CompactFrameID target_id = lookupFrameNumber(target_frame);
      TimeCacheInterfacePtr cache = getFrame(target_id);
      if(cache){
        if(cc == TwoPhaseLock){
          frame_rw_lock_[target_id].r_lock();
          identity.header.stamp = cache->getLatestTimestamp();
          frame_rw_lock_[target_id].r_unlock();
        } else if(cc == Silo){
        retry:
          auto old_v = frame_vrw_lock_[target_id].virtualRLock();
          identity.header.stamp = cache->getLatestTimestamp();
          auto new_v = frame_vrw_lock_[target_id].virtualRLock();
          if(old_v != new_v) goto retry;
        }
      }else{
        identity.header.stamp = ros::Time(0);
      }
      if(stat != nullptr){
        stat->timestamps.push_back(identity.header.stamp.toNSec());
      }

      return identity;
    }

    //Identify case does not need to be validated above
    CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
    CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);
    assert(target_id != source_id);

    std::string error_string;
    TransformAccum accum;
    int retval = walkToTopParentLatest(accum, target_id, source_id, &error_string, stat);
    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
        case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw ConnectivityException(error_string);
        case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw ExtrapolationException(error_string);
        case tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw LookupException(error_string);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
          assert(0);
      }
    }

    geometry_msgs::TransformStamped output_transform;
    transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform, accum.time, target_frame, source_frame);
    return output_transform;
  }

  void BufferCore::justReadFrames(const std::vector<std::string> &frames, ReadStat *stat) const{
    tf2::TransformStorage st{};
    ScopedWriteSetUnLocker un_locker(frame_rw_lock_);
    ReadChecker read_checker(frame_vrw_lock_);
    
  retry:
    for(auto &frame_str: frames){
      auto frame_id = lookupFrameNumber(frame_str);

      // just allow to fail insert
      if(frame_id != 0){
        auto frame = getFrame(frame_id);
        if(frame != nullptr){ 
          if(cc == TwoPhaseLock){
            un_locker.rLockIfNot(frame_id);
          }else if(cc == Silo){
            read_checker.addRLock(frame_id);
          }
          frame->getData(ros::Time(0), st, nullptr);
          
          if(stat){
            stat->timestamps.push_back(st.stamp_.toNSec());
          }
        }
      }
    }
    
    if(cc == Silo and !read_checker.check()){
      if(stat){
        stat->timestamps.clear();
      }
      read_checker.clear();
      goto retry;
    }
  }

  geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame,
                                                              const ros::Time& target_time,
                                                              const std::string& source_frame,
                                                              const ros::Time& source_time,
                                                              const std::string& fixed_frame) const noexcept(false)
  {
    validateFrameId("lookupTransform argument target_frame", target_frame);
    validateFrameId("lookupTransform argument source_frame", source_frame);
    validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

    geometry_msgs::TransformStamped output;
    geometry_msgs::TransformStamped temp1 =  lookupTransform(fixed_frame, source_frame, source_time);
    geometry_msgs::TransformStamped temp2 =  lookupTransform(target_frame, fixed_frame, target_time);

    tf2::Transform tf1, tf2;
    transformMsgToTF2(temp1.transform, tf1);
    transformMsgToTF2(temp2.transform, tf2);
    transformTF2ToMsg(tf2*tf1, output.transform);
    output.header.stamp = temp2.header.stamp;
    output.header.frame_id = target_frame;
    output.child_frame_id = source_frame;
    return output;
  }



/*
geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame,
                                          const std::string& observation_frame,
                                          const ros::Time& time,
                                          const ros::Duration& averaging_interval) const
{
  try
  {
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame,
                                          const std::string& observation_frame,
                                          const std::string& reference_frame,
                                          const tf2::Point & reference_point,
                                          const std::string& reference_point_frame,
                                          const ros::Time& time,
                                          const ros::Duration& averaging_interval) const
{
  try{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}
*/

  struct CanTransformAccum
  {
    CompactFrameID gather(TimeCacheInterfacePtr cache, ros::Time time, std::string* error_string, ReadStat *stat = nullptr)
    {
      return cache->getParent(time, error_string);
    }

    void accum(bool source)
    {
    }

    void finalize(WalkEnding end, ros::Time _time)
    {
    }

    TransformStorage st;
  };

  bool BufferCore::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                                      const ros::Time& time, std::string* error_msg) const noexcept(false)
  {
    if (target_id == 0 || source_id == 0)
    {
      if (error_msg)
      {
        if (target_id == 0)
        {
          *error_msg += std::string("target_frame: " + lookupFrameString(target_id ) + " does not exist.");
        }
        if (source_id == 0)
        {
          if (target_id == 0)
          {
            *error_msg += std::string(" ");
          }
          *error_msg += std::string("source_frame: " + lookupFrameString(source_id) + " " + lookupFrameString(source_id ) + " does not exist.");
        }
      }
      return false;
    }

    if (target_id == source_id)
    {
      return true;
    }

    CanTransformAccum accum;
    if (walkToTopParent(accum, time, target_id, source_id, error_msg) == tf2_msgs::TF2Error::NO_ERROR)
    {
      return true;
    }

    return false;
  }

  bool BufferCore::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                                        const ros::Time& time, std::string* error_msg) const noexcept(false)
  {
    return canTransformNoLock(target_id, source_id, time, error_msg);
  }

  bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                                const ros::Time& time, std::string* error_msg) const noexcept(false)
  {
    // Short circuit if target_frame == source_frame
    if (target_frame == source_frame)
      return true;

    if (warnFrameId("canTransform argument target_frame", target_frame))
      return false;
    if (warnFrameId("canTransform argument source_frame", source_frame))
      return false;

    CompactFrameID target_id = lookupFrameNumber(target_frame);
    CompactFrameID source_id = lookupFrameNumber(source_frame);

    if (target_id == 0 || source_id == 0)
    {
      if (error_msg)
      {
        if (target_id == 0)
        {
          *error_msg += std::string("canTransform: target_frame " + target_frame + " does not exist.");
        }
        if (source_id == 0)
        {
          if (target_id == 0)
          {
            *error_msg += std::string(" ");
          }
          *error_msg += std::string("canTransform: source_frame " + source_frame + " does not exist.");
        }
      }
      return false;
    }
    return canTransformNoLock(target_id, source_id, time, error_msg);
  }

  bool BufferCore::canTransform(const std::string& target_frame, const ros::Time& target_time,
                                const std::string& source_frame, const ros::Time& source_time,
                                const std::string& fixed_frame, std::string* error_msg) const noexcept(false)
  {
    if (warnFrameId("canTransform argument target_frame", target_frame))
      return false;
    if (warnFrameId("canTransform argument source_frame", source_frame))
      return false;
    if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
      return false;

    CompactFrameID target_id = lookupFrameNumber(target_frame);
    CompactFrameID source_id = lookupFrameNumber(source_frame);
    CompactFrameID fixed_id = lookupFrameNumber(fixed_frame);

    if (target_id == 0 || source_id == 0 || fixed_id == 0)
    {
      if (error_msg)
      {
        if (target_id == 0)
        {
          *error_msg += std::string("canTransform: target_frame " + target_frame + " does not exist.");
        }
        if (source_id == 0)
        {
          if (target_id == 0)
          {
            *error_msg += std::string(" ");
          }
          *error_msg += std::string("canTransform: source_frame " + source_frame + " does not exist.");
        }
        if (source_id == 0)
        {
          if (target_id == 0 || source_id == 0)
          {
            *error_msg += std::string(" ");
          }
          *error_msg += std::string("fixed_frame: " + fixed_frame + "does not exist.");
        }
      }
      return false;
    }
    return canTransformNoLock(target_id, fixed_id, target_time, error_msg) && canTransformNoLock(fixed_id, source_id, source_time, error_msg);
  }

  tf2::TimeCacheInterfacePtr BufferCore::getFrame(CompactFrameID frame_id) const noexcept
  {
    if (frame_id >= next_frame_id_)
      return nullptr;
    else
    {
      return &frames_[frame_id];
    }
  }

  CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const noexcept
  {
    CompactFrameID retval;
    auto map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end())
    {
      retval = CompactFrameID(0);
    }
    else
      retval = map_it->second;
    return retval;
  }

  CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str) noexcept
  {
    CompactFrameID retval = 0;
    auto map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end()){
      retval = next_frame_id_.fetch_add(1);
      frameIDs_[frameid_str] = retval;
      frameIDs_reverse[retval] = frameid_str;
    }
    else
      retval = frameIDs_[frameid_str];

    return retval;
  }

  const std::string& BufferCore::lookupFrameString(CompactFrameID frame_id_num) const noexcept(false)
  {
    if (frame_id_num >= next_frame_id_)
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];
  }

  void BufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const noexcept(false)
  {
    if (!out)
    {
      return;
    }
    *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                       lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                       "Tf has two or more unconnected trees.");
  }

  std::string BufferCore::allFramesAsString() const noexcept
  {
    return this->allFramesAsStringNoLock();
  }

  std::string BufferCore::allFramesAsStringNoLock() const noexcept
  {
    std::stringstream mstream;

    TransformStorage temp;

    ///regular transforms
    for (unsigned int counter = 1; counter < next_frame_id_; counter ++)
    {
      TimeCacheInterfacePtr frame_ptr = getFrame(CompactFrameID(counter));
      if (frame_ptr == nullptr)
        continue;
      CompactFrameID frame_id_num;

      bool result;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[counter].r_lock();
        result = frame_ptr->getData(ros::Time(), temp);
        frame_rw_lock_[counter].r_unlock();
      } else if(cc == Silo){
      retry:
        auto old_v = frame_vrw_lock_[counter].virtualRLock();
        result = frame_ptr->getData(ros::Time(), temp);
        auto new_v = frame_vrw_lock_[counter].virtualRLock();
        if(old_v != new_v) goto retry;
      }

      if(result)
        frame_id_num = temp.frame_id_;
      else
      {
        frame_id_num = 0;
      }
      mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
    }

    return mstream.str();
  }

  struct TimeAndFrameIDFrameComparator
  {
    TimeAndFrameIDFrameComparator(CompactFrameID id)
      : id(id)
    {}

    bool operator()(const P_TimeAndFrameID& rhs) const
    {
      return rhs.second == id;
    }

    CompactFrameID id;
  };

  int BufferCore::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, ros::Time & time, std::string * error_string) const noexcept
  {
    // Error if one of the frames don't exist.
    if (source_id == 0 || target_id == 0) return tf2_msgs::TF2Error::LOOKUP_ERROR;

    if (source_id == target_id)
    {
      TimeCacheInterfacePtr cache = getFrame(source_id);
      //Set time to latest timestamp of frameid in case of target and source frame id are the same
      if (cache) {
        if(cc == TwoPhaseLock){
          frame_rw_lock_[source_id].r_lock();
          time = cache->getLatestTimestamp();
          frame_rw_lock_[source_id].r_unlock();
        }else if(cc == Silo){
        retry:
          auto old_v = frame_vrw_lock_[source_id].virtualRLock();
          time = cache->getLatestTimestamp();
          auto new_v = frame_vrw_lock_[source_id].virtualRLock();
          if(old_v != new_v) goto retry;
        }
      }else
        time = ros::Time();
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    std::vector<P_TimeAndFrameID> lct_cache;

    // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
    // in the target is a direct parent
    CompactFrameID frame = source_id;
    P_TimeAndFrameID temp;
    uint32_t depth = 0;
    ros::Time common_time = ros::TIME_MAX;
    while (frame != 0)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);

      if (!cache)
      {
        // There will be no cache for the very root of the tree
        break;
      }

      P_TimeAndFrameID latest;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[frame].r_lock();
        latest = cache->getLatestTimeAndParent();
        frame_rw_lock_[frame].r_unlock();
      }else if(cc == Silo){
      retry2:
        auto old_v = frame_vrw_lock_[frame].virtualRLock();
        latest = cache->getLatestTimeAndParent();
        auto new_v = frame_vrw_lock_[frame].virtualRLock();
        if(old_v != new_v) goto retry2;
      }

      if (latest.second == 0)
      {
        // Just break out here... there may still be a path from source -> target
        break;
      }

      if (!latest.first.isZero())
      {
        common_time = std::min(latest.first, common_time);
      }

      lct_cache.push_back(latest);

      frame = latest.second;

      // Early out... target frame is a direct parent of the source frame
      if (frame == target_id)
      {
        time = common_time;
        if (time == ros::TIME_MAX)
        {
          time = ros::Time();
        }
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
    frame = target_id;
    depth = 0;
    common_time = ros::TIME_MAX;
    CompactFrameID common_parent = 0;
    while (true)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);

      if (!cache)
      {
        break;
      }

      P_TimeAndFrameID latest;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[frame].r_lock();
        latest = cache->getLatestTimeAndParent();
        frame_rw_lock_[frame].r_unlock();
      }else if(cc == Silo){
      retry3:
        auto old_v = frame_vrw_lock_[frame].virtualRLock();
        latest = cache->getLatestTimeAndParent();
        auto new_v = frame_vrw_lock_[frame].virtualRLock();
        if(old_v != new_v) goto retry3;
      }

      if (latest.second == 0)
      {
        break;
      }

      if (!latest.first.isZero())
      {
        common_time = std::min(latest.first, common_time);
      }

      std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
      if (it != lct_cache.end()) // found a common parent
      {
        common_parent = it->second;
        break;
      }

      frame = latest.second;

      // Early out... source frame is a direct parent of the target frame
      if (frame == source_id)
      {
        time = common_time;
        if (time == ros::TIME_MAX)
        {
          time = ros::Time();
        }
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      ++depth;
      if (depth > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsStringNoLock() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    if (common_parent == 0)
    {
      createConnectivityErrorString(source_id, target_id, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    // Loop through the source -> root list until we hit the common parent
    {
      auto it = lct_cache.begin();
      auto end = lct_cache.end();
      for (; it != end; ++it)
      {
        if (!it->first.isZero())
        {
          common_time = std::min(common_time, it->first);
        }

        if (it->second == common_parent)
        {
          break;
        }
      }
    }

    if (common_time == ros::TIME_MAX)
    {
      common_time = ros::Time();
    }

    time = common_time;
    return tf2_msgs::TF2Error::NO_ERROR;
  }

  std::string BufferCore::allFramesAsYAML(double current_time) const noexcept
  {
    std::stringstream mstream;

    TransformStorage temp;

    if (next_frame_id_==1)
      mstream <<"{}";

    mstream.precision(3);
    mstream.setf(std::ios::fixed,std::ios::floatfield);

    //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
    for (unsigned int counter = 1; counter < next_frame_id_; counter ++)//one referenced for 0 is no frame
    {
      CompactFrameID cfid = CompactFrameID(counter);
      CompactFrameID frame_id_num;
      TimeCacheInterfacePtr cache = getFrame(cfid);
      if (!cache)
      {
        continue;
      }

retry:
      uint32_t old_v;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[cfid].r_lock();
      }else if(cc == Silo){
        old_v = frame_vrw_lock_[cfid].virtualRLock();
      }

      if(!cache->getData(ros::Time(), temp))
      {
        continue;
      }

      frame_id_num = temp.frame_id_;

      std::string authority = "no recorded authority";
      authority = frame_authority_[cfid];

      double rate = cache->getListLength() / std::max((cache->getLatestTimestamp().toSec() -
                                                       cache->getOldestTimestamp().toSec() ), 0.0001);

      std::stringstream tmp;
      tmp << std::fixed; //fixed point notation
      tmp.precision(3); //3 decimal places
      tmp << frameIDs_reverse[cfid] << ": " << std::endl;
      tmp << "  parent: '" << frameIDs_reverse[frame_id_num] << "'" << std::endl;
      tmp << "  broadcaster: '" << authority << "'" << std::endl;
      tmp << "  rate: " << rate << std::endl;
      tmp << "  most_recent_transform: " << (cache->getLatestTimestamp()).toSec() << std::endl;
      tmp << "  oldest_transform: " << (cache->getOldestTimestamp()).toSec() << std::endl;
      if ( current_time > 0 ) {
        tmp << "  transform_delay: " << current_time - cache->getLatestTimestamp().toSec() << std::endl;
      }
      tmp << "  buffer_length: " << (cache->getLatestTimestamp() - cache->getOldestTimestamp()).toSec() << std::endl;

      if(cc == TwoPhaseLock){
        frame_rw_lock_[cfid].r_unlock();
      }else if(cc == Silo){
        auto new_v = frame_vrw_lock_[cfid].virtualRLock();
        if(old_v != new_v) goto retry;
      }

      mstream << tmp.str();
    }

    return mstream.str();
  }

  std::string BufferCore::allFramesAsYAML() const noexcept
  {
    return this->allFramesAsYAML(0.0);
  }

  TransformableCallbackHandle BufferCore::addTransformableCallback(const TransformableCallback& cb)
  {
    boost::mutex::scoped_lock lock(transformable_callbacks_mutex_);
    TransformableCallbackHandle handle = ++transformable_callbacks_counter_;
    while (!transformable_callbacks_.insert(std::make_pair(handle, cb)).second)
    {
      handle = ++transformable_callbacks_counter_;
    }

    return handle;
  }

  struct BufferCore::RemoveRequestByCallback
  {
    RemoveRequestByCallback(TransformableCallbackHandle handle)
      : handle_(handle)
    {}

    bool operator()(const TransformableRequest& req)
    {
      return req.cb_handle == handle_;
    }

    TransformableCallbackHandle handle_;
  };

  void BufferCore::removeTransformableCallback(TransformableCallbackHandle handle)
  {
    {
      boost::mutex::scoped_lock lock(transformable_callbacks_mutex_);
      transformable_callbacks_.erase(handle);
    }

    {
      boost::mutex::scoped_lock lock(transformable_requests_mutex_);
      auto it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByCallback(handle));
      transformable_requests_.erase(it, transformable_requests_.end());
    }
  }

  TransformableRequestHandle BufferCore::addTransformableRequest(TransformableCallbackHandle handle, const std::string& target_frame, const std::string& source_frame, ros::Time time)
  {
    // shortcut if target == source
    if (target_frame == source_frame)
    {
      return 0;
    }

    TransformableRequest req;
    req.target_id = lookupFrameNumber(target_frame);
    req.source_id = lookupFrameNumber(source_frame);

    // First check if the request is already transformable.  If it is, return immediately
    if (canTransformInternal(req.target_id, req.source_id, time, 0))
    {
      return 0;
    }

    // Might not be transformable at all, ever (if it's too far in the past)
    if (req.target_id && req.source_id)
    {
      ros::Time latest_time;
      // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
      // any of the frames
      getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
      if (!latest_time.isZero() && time + cache_time_ < latest_time)
      {
        return 0xffffffffffffffffULL;
      }
    }

    req.cb_handle = handle;
    req.time = time;
    req.request_handle = ++transformable_requests_counter_;
    if (req.request_handle == 0 || req.request_handle == 0xffffffffffffffffULL)
    {
      req.request_handle = 1;
    }

    if (req.target_id == 0)
    {
      req.target_string = target_frame;
    }

    if (req.source_id == 0)
    {
      req.source_string = source_frame;
    }

    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    transformable_requests_.push_back(req);

    return req.request_handle;
  }

  struct BufferCore::RemoveRequestByID
  {
    RemoveRequestByID(TransformableRequestHandle handle)
      : handle_(handle)
    {}

    bool operator()(const TransformableRequest& req)
    {
      return req.request_handle == handle_;
    }

    TransformableCallbackHandle handle_;
  };

  void BufferCore::cancelTransformableRequest(TransformableRequestHandle handle)
  {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    auto it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByID(handle));

    if (it != transformable_requests_.end())
    {
      transformable_requests_.erase(it, transformable_requests_.end());
    }
  }



// backwards compability for tf methods
  boost::signals2::connection BufferCore::_addTransformsChangedListener(boost::function<void(void)> callback)
  {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    return _transforms_changed_.connect(callback);
  }

  void BufferCore::_removeTransformsChangedListener(boost::signals2::connection c)
  {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    c.disconnect();
  }

  bool BufferCore::_frameExists(const std::string& frame_id_str) const
  {
    return frameIDs_.count(frame_id_str);
  }

  bool BufferCore::_getParent(const std::string& frame_id, ros::Time time, std::string& parent) const
  {

    CompactFrameID frame_number = lookupFrameNumber(frame_id);
    TimeCacheInterfacePtr frame = getFrame(frame_number);

    if (!frame)
      return false;

    CompactFrameID parent_id;
    if(cc == TwoPhaseLock){
      frame_rw_lock_[frame_number].r_lock();
      parent_id = frame->getParent(time, nullptr);
      frame_rw_lock_[frame_number].r_unlock();
    }else if(cc == Silo){
    retry:
      auto old_v = frame_vrw_lock_[frame_number].virtualRLock();
      parent_id = frame->getParent(time, nullptr);
      auto new_v = frame_vrw_lock_[frame_number].virtualRLock();
      if(old_v != new_v) goto retry;
    }

    if (parent_id == 0)
      return false;

    parent = lookupFrameString(parent_id);
    return true;
  };

  void BufferCore::_getFrameStrings(std::vector<std::string> & vec) const
  {
    vec.clear();

    TransformStorage temp;

    //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
    for (unsigned int counter = 1; counter < next_frame_id_; counter ++)
    {
      vec.push_back(frameIDs_reverse[counter]);
    }
    return;
  }

  void BufferCore::testTransformableRequests()
  {
    if(transformable_requests_.empty()){
      return;
    }

    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    auto it = transformable_requests_.begin();

    typedef boost::tuple<TransformableCallback&, TransformableRequestHandle, std::string,
      std::string, ros::Time&, TransformableResult&> TransformableTuple;
    std::vector<TransformableTuple> transformables;

    for (; it != transformable_requests_.end();)
    {
      TransformableRequest& req = *it;

      // One or both of the frames may not have existed when the request was originally made.
      if (req.target_id == 0)
      {
        req.target_id = lookupFrameNumber(req.target_string);
      }

      if (req.source_id == 0)
      {
        req.source_id = lookupFrameNumber(req.source_string);
      }

      ros::Time latest_time;
      bool do_cb = false;
      TransformableResult result = TransformAvailable;
      // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
      // any of the frames
      getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
      if (!latest_time.isZero() && req.time + cache_time_ < latest_time)
      {
        do_cb = true;
        result = TransformFailure;
      }
      else if (canTransformInternal(req.target_id, req.source_id, req.time, 0))
      {
        do_cb = true;
        result = TransformAvailable;
      }

      if (do_cb)
      {
        {
          boost::mutex::scoped_lock lock2(transformable_callbacks_mutex_);
          M_TransformableCallback::iterator it = transformable_callbacks_.find(req.cb_handle);
          if (it != transformable_callbacks_.end())
          {
            transformables.push_back(boost::make_tuple(boost::ref(it->second),
                                                       req.request_handle,
                                                       lookupFrameString(req.target_id),
                                                       lookupFrameString(req.source_id),
                                                       boost::ref(req.time),
                                                       boost::ref(result)));
          }
        }

        if (transformable_requests_.size() > 1)
        {
          transformable_requests_[it - transformable_requests_.begin()] = transformable_requests_.back();
        }

        transformable_requests_.erase(transformable_requests_.end() - 1);
      }
      else
      {
        ++it;
      }
    }

    // unlock before allowing possible user callbacks to avoid potential deadlock (#91)
    lock.unlock();

    BOOST_FOREACH (TransformableTuple tt, transformables)
          {
            tt.get<0>()(tt.get<1>(), tt.get<2>(), tt.get<3>(), tt.get<4>(), tt.get<5>());
          }

    // Backwards compatability callback for tf
    _transforms_changed_();
  }

  std::string BufferCore::_allFramesAsDot(double current_time) const
  {
    std::stringstream mstream;
    mstream << "digraph G {" << std::endl;

    TransformStorage temp;

    if (next_frame_id_ == 1) {
      mstream <<"\"no tf data recieved\"";
    }
    mstream.precision(3);
    mstream.setf(std::ios::fixed,std::ios::floatfield);

    for (unsigned int counter = 1; counter < next_frame_id_; counter ++) // one referenced for 0 is no frame
    {
      unsigned int frame_id_num;
      TimeCacheInterfacePtr counter_frame = getFrame(counter);
      if (!counter_frame) {
        continue;
      }
    retry:
      uint32_t old_v;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[counter].r_lock();
      }else if(cc == Silo){
        old_v = frame_vrw_lock_[counter].virtualRLock();
      }

      if(!counter_frame->getData(ros::Time(), temp)) {
        continue;
      } else {
        frame_id_num = temp.frame_id_;
      }
      std::string authority = "no recorded authority";
      authority = frame_authority_[counter];

      double rate = counter_frame->getListLength() / std::max((counter_frame->getLatestTimestamp().toSec() -
                                                               counter_frame->getOldestTimestamp().toSec()), 0.0001);

      std::stringstream tmp;
      tmp << std::fixed; //fixed point notation
      tmp.precision(3); //3 decimal places
      tmp << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
              << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
              //<< "Time: " << current_time.toSec() << "\\n"
              << "Broadcaster: " << authority << "\\n"
              << "Average rate: " << rate << " Hz\\n"
              << "Most recent transform: " << (counter_frame->getLatestTimestamp()).toSec() <<" ";
      if (current_time > 0)
        tmp << "( "<<  current_time - counter_frame->getLatestTimestamp().toSec() << " sec old)";
      tmp << "\\n"
              //    << "(time: " << getFrame(counter)->getLatestTimestamp().toSec() << ")\\n"
              //    << "Oldest transform: " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec() << " sec old \\n"
              //    << "(time: " << (getFrame(counter)->getOldestTimestamp()).toSec() << ")\\n"
              << "Buffer length: " << (counter_frame->getLatestTimestamp()-counter_frame->getOldestTimestamp()).toSec() << " sec\\n"
              <<"\"];" <<std::endl;

      if(cc == TwoPhaseLock){
        frame_rw_lock_[counter].r_unlock();
      }else if(cc == Silo){
        auto new_v = frame_vrw_lock_[counter].virtualRLock();
        if(old_v != new_v) goto retry;
      }

      mstream << tmp.str();
    }

    for (unsigned int counter = 1; counter < next_frame_id_; counter ++)//one referenced for 0 is no frame
    {
      unsigned int frame_id_num;
      TimeCacheInterfacePtr counter_frame = getFrame(counter);
      if (!counter_frame) {
        if (current_time > 0) {
          mstream << "edge [style=invis];" <<std::endl;
          mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
                  << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n "
                  << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
        }
        continue;
      }

    retry2:
      uint32_t old_v;
      if(cc == TwoPhaseLock){
        frame_rw_lock_[counter].r_lock();
      }else if(cc == Silo){
        old_v = frame_vrw_lock_[counter].virtualRLock();
      }

      if (counter_frame->getData(ros::Time(), temp)) {
        frame_id_num = temp.frame_id_;
      } else {
        frame_id_num = 0;
      }

      std::stringstream tmp;
      if(frameIDs_reverse[frame_id_num]=="NO_PARENT")
      {
        tmp << "edge [style=invis];" <<std::endl;
        tmp << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n";
        if (current_time > 0)
          tmp << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n ";
        tmp << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
      }

      if(cc == TwoPhaseLock){
        frame_rw_lock_[counter].r_unlock();
      }else if(cc == Silo){
        auto new_v = frame_vrw_lock_[counter].virtualRLock();
        if(old_v != new_v) goto retry2;
      }
      mstream << tmp.str();
    }
    mstream << "}";
    return mstream.str();
  }

  std::string BufferCore::_allFramesAsDot() const
  {
    return _allFramesAsDot(0.0);
  }

  void BufferCore::_chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
  {
    std::string error_string;

    output.clear(); //empty vector

    std::stringstream mstream;

    TransformAccum accum;

    // Get source frame/time using getFrame
    CompactFrameID source_id = lookupFrameNumber(source_frame);
    CompactFrameID fixed_id = lookupFrameNumber(fixed_frame);
    CompactFrameID target_id = lookupFrameNumber(target_frame);

    std::vector<CompactFrameID> source_frame_chain;
    int retval = walkToTopParent(accum, source_time, fixed_id, source_id, &error_string, &source_frame_chain);

    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
        case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw ConnectivityException(error_string);
        case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw ExtrapolationException(error_string);
        case tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw LookupException(error_string);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
          assert(0);
      }
    }

    std::vector<CompactFrameID> target_frame_chain;
    retval = walkToTopParent(accum, target_time, target_id, fixed_id, &error_string, &target_frame_chain);

    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
        case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw ConnectivityException(error_string);
        case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw ExtrapolationException(error_string);
        case tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw LookupException(error_string);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
          assert(0);
      }
    }
    // If the two chains overlap clear the overlap
    if (source_frame_chain.size() > 0 && target_frame_chain.size() > 0 &&
        source_frame_chain.back() == target_frame_chain.front())
    {
      source_frame_chain.pop_back();
    }
    // Join the two walks
    for (unsigned int i = 0; i < target_frame_chain.size(); ++i)
    {
      source_frame_chain.push_back(target_frame_chain[i]);
    }


    // Write each element of source_frame_chain as string
    for (unsigned int i = 0; i < source_frame_chain.size(); ++i)
    {
      output.push_back(lookupFrameString(source_frame_chain[i]));
    }
  }

  int TestBufferCore::_walkToTopParent(BufferCore& buffer, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID> *frame_chain) const
  {
    TransformAccum accum;
    return buffer.walkToTopParent(accum, time, target_id, source_id, error_string, frame_chain);
  }
} // namespace tf2