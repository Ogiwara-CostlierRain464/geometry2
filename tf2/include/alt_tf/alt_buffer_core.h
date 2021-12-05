#ifndef ALT_TF_ALT_BUFFER_CORE_H
#define ALT_TF_ALT_BUFFER_CORE_H

#include <ros/duration.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <unordered_map>
#include <console_bridge/console.h>

#include "../tf2/time_cache.h"
#include "tf2/rwlock.h"
#include "../tf2/transform_storage.h"
#include "../tf2/error.h"
#include "../tf2/exceptions.h"

namespace alt_tf{

typedef std::pair<ros::Time, tf2::CompactFrameID> TimeAndFrameID;
typedef std::shared_ptr<tf2::TimeCacheInterface> TimeCacheInterfacePtr;
typedef std::unique_ptr<RWLock> RWLockPtr;

enum TransformableResult
{
  TransformAvailable,
  TransformFailure,
};


  enum WalkEnding{
    Identity,
    TargetParentOfSource,
    SourceParentOfTarget,
    FullPath
  };

// while looking up tree, accum information
struct TransformAccum{
  tf2::TransformStorage st;
  ros::Time time;
  tf2::Quaternion sourceToTopQuat;
  tf2::Vector3 sourceToTopVec;
  tf2::Quaternion targetToTopQuat;
  tf2::Vector3 targetToTopVec;
  tf2::Quaternion resultQuat;
  tf2::Vector3 resultVec;

  TransformAccum()
    : sourceToTopQuat(0, 0, 0, 1)
    , sourceToTopVec(0, 0, 0)
    , targetToTopQuat(0, 0, 0, 1)
    , targetToTopVec(0, 0, 0)
    , resultQuat(0, 0, 0, 1)
    , resultVec(0, 0, 0){}

  tf2::CompactFrameID gather(const TimeCacheInterfacePtr& cache,
                        const ros::Time &time_,
                        std::string *err_str){
    if(!cache->getData(time_, st, err_str)){
      return 0;
    }
    return st.frame_id_;
  }

  void accum(bool source){
    if(source){
      sourceToTopVec =
        tf2::quatRotate(st.rotation_, sourceToTopVec) + st.translation_;
      sourceToTopQuat *= st.rotation_;
    }else{
      targetToTopVec =
        tf2::quatRotate(st.rotation_, targetToTopVec) + st.translation_;
      targetToTopQuat *= st.rotation_;
    }
  }

  void finalize(WalkEnding end, const ros::Time time_){
    switch (end) {
      case Identity:
        break;
      case TargetParentOfSource:
        resultVec = sourceToTopVec;
        resultQuat = sourceToTopQuat;
        break;
      case SourceParentOfTarget: {
        tf2::Quaternion inv_target_quat = targetToTopQuat.inverse();
        auto inv_target_vec =
          tf2::quatRotate(inv_target_quat, -targetToTopVec);
        resultVec = inv_target_vec;
        resultQuat = inv_target_quat;
        break;
      }
      case FullPath: {
        auto inv_target_quat = targetToTopQuat.inverse();
        auto inv_target_vec = tf2::quatRotate(inv_target_quat, -targetToTopVec);
        resultVec = tf2::quatRotate(inv_target_quat, sourceToTopVec) + inv_target_vec;
        resultQuat = inv_target_quat * sourceToTopQuat;
        break;
      }
    }

    time = time_;
  }
};

class ScopedWriteUnLocker{
public:
  ScopedWriteUnLocker(
    std::set<tf2::CompactFrameID> &set,
    std::vector<RWLockPtr> &frameMutex_)
    : lockedIdSet(set), frameMutex(frameMutex_){}

  ~ScopedWriteUnLocker(){
    for(auto id : lockedIdSet){
      frameMutex[id]->w_unlock();
    }
  }

private:
  std::set<tf2::CompactFrameID> &lockedIdSet;
  std::vector<RWLockPtr> &frameMutex;
};

class ScopedReadUnLocker{
public:
  ScopedReadUnLocker(
    std::set<tf2::CompactFrameID> &set,
    std::vector<RWLockPtr> &frameMutex_)
    : lockedIdSet(set), frameMutex(frameMutex_){}

  void rLockIfItWasNot(tf2::CompactFrameID id){
    if(lockedIdSet.find(id) == lockedIdSet.end()){
      frameMutex[id]->r_lock();
      lockedIdSet.insert(id);
    }
  }

  ~ScopedReadUnLocker(){
    for(auto id : lockedIdSet){
      assert(frameMutex[id]->isLocked());
      frameMutex[id]->r_unlock();
    }
  }

private:
  std::set<tf2::CompactFrameID> &lockedIdSet;
  std::vector<RWLockPtr> &frameMutex;
};

class ScopedTreeReadUnLocker{
public:
  ScopedTreeReadUnLocker(RWLock &lock_)
  :lock(lock_){}

  void rLock(){
    lock.r_lock();
  }

  ~ScopedTreeReadUnLocker(){
    lock.r_unlock();
  }
private:
  RWLock &lock;
};

class AltBufferCore{
public:
  static const int DEFAULT_CACHE_TIME = 10;  //!< The default amount of time to cache data in seconds
  static const uint32_t MAX_GRAPH_DEPTH = 1000UL;  //!< Maximum graph search depth (deeper graphs will be assumed to have loops)
  constexpr static const double QUATERNION_NORMALIZATION_TOLERANCE = 10e-3;

  AltBufferCore(ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME))
  : cacheTime(cache_time_)
  {
    frameToId["NO_PARENT"] = 0;
    frames.emplace_back();
    idToFrame.push_back("NO_PARENT");
    frameIdMutex.emplace_back(); // no need to alloc.
  }

private:
  TimeCacheInterfacePtr getFrame(tf2::CompactFrameID frame_id) const{
    if(frame_id >= frames.size()){
      // not throwing error?
      return TimeCacheInterfacePtr();
    }else{
      return frames[frame_id];
    }
  }

  tf2::CompactFrameID lookupFrameNumber(
    const std::string &frame
    )const{
    assert(treeMutex.isLocked());

    auto map_it = frameToId.find(frame);
    if(map_it == frameToId.end()){
      return tf2::CompactFrameID(0);
    }else{
      return map_it->second;
    }
  }

  tf2::CompactFrameID lookupOrInsertFrameNumber(
    const std::string &frame,
    bool &lock_upgraded
    ) noexcept{
    assert(treeMutex.isLocked());
    tf2::CompactFrameID ret{};
    auto map_it = frameToId.find(frame);
    if(map_it == frameToId.end()){
      if(not lock_upgraded){
        treeMutex.upgrade();
        lock_upgraded = true;
      }
      ret = tf2::CompactFrameID(frames.size());
      frames.push_back(TimeCacheInterfacePtr());
      frameIdMutex.emplace_back(std::make_unique<RWLock>());
      frameToId[frame] = ret;
      idToFrame.push_back(frame);
    }else{
      ret = frameToId[frame];
    }
    return ret;
  }

  TimeCacheInterfacePtr allocateFrame(tf2::CompactFrameID id, bool is_static) noexcept{
    auto frame_ptr = frames[id];
    if(is_static){
      assert(false && "Not implemented");
    }else{
      frames[id] = std::make_shared<tf2::TimeCache>(cacheTime);
    }

    return frames[id];
  }

  static bool startsWithSlash(const std::string& frame){
    return !frame.empty() and frame[0] == '/';
  }

  static std::string stripSlash(const std::string &in){
    std::string out = in;
    if (startsWithSlash(in))
      out.erase(0,1);
    return out;
  }

  static void transformTF2ToMsg(
    const tf2::Quaternion &orient,
    const tf2::Vector3 &pos,
    geometry_msgs::Transform &out_msg
    ){
    out_msg = geometry_msgs::Transform{};
    out_msg.translation.x = pos.x();
    out_msg.translation.y = pos.y();
    out_msg.translation.z = pos.z();
    out_msg.rotation.x = orient.x();
    out_msg.rotation.y = orient.y();
    out_msg.rotation.z = orient.z();
    out_msg.rotation.w = orient.w();
  }

  static void transformTF2ToMsg(
    const tf2::Quaternion &orient,
    const tf2::Vector3 &pos,
    geometry_msgs::TransformStamped &out_msg,
    const ros::Time &stamp,
    const std::string &frame,
    const std::string &child_frame
    ){
    out_msg = geometry_msgs::TransformStamped{};
    transformTF2ToMsg(orient, pos, out_msg.transform);
    out_msg.header.stamp = stamp;
    out_msg.header.frame_id = frame;
    out_msg.child_frame_id = child_frame;
  }

  tf2::CompactFrameID validateAndGetFrameId(
    const char *function_name_arg,
    const std::string &frame
    )const{
    if(frame.empty()){
      std::stringstream ss;
      ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
      throw tf2::InvalidArgumentException(ss.str().c_str());
    }
    if(startsWithSlash(frame)){
      std::stringstream ss;
      ss << "Invalid argument \"" << frame << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
      throw tf2::InvalidArgumentException(ss.str().c_str());
    }

    tf2::CompactFrameID id = lookupFrameNumber(frame);
    if (id == 0)
    {
      std::stringstream ss;
      ss << "\"" << frame << "\" passed to "<< function_name_arg <<" does not exist. ";
      throw tf2::LookupException(ss.str().c_str());
    }
    return id;
  }

  void createConnectivityErrorString(
    tf2::CompactFrameID source_frame_id,
    tf2::CompactFrameID target_frame_id,
    std::string *out
    )const{
    if(!out){
      return;
    }
    *out = std::string("Could not find a connection");
  }

  struct TimeAndFrameIDComparator{
    tf2::CompactFrameID id;

    TimeAndFrameIDComparator(tf2::CompactFrameID id)
    : id(id){}

    bool operator()(const TimeAndFrameID &rhs) const{
      return rhs.second == id;
    }
  };

  tf2_msgs::TF2Error getLatestCommonTime(
    tf2::CompactFrameID target_id,
    tf2::CompactFrameID source_id,
    ScopedReadUnLocker &un_locker,
    ros::Time &out_time,
    std::string *err_str
    ) const {
    // looking up and locking up tree to find latest common time
    // this method only do locks and does not unlock.
    if(source_id == 0 or target_id == 0){
      return tf2_msgs::LOOKUP_ERROR;
    }

    // short circuit
    if(source_id == target_id){ // s == t
      auto cache = getFrame(source_id);
      if(cache){
        un_locker.rLockIfItWasNot(source_id);
        out_time = cache->getLatestTimestamp();
      }else{
        out_time = ros::Time();
      }
      return tf2_msgs::NO_ERROR;
    }

    std::vector<TimeAndFrameID> lookup_cache;

    // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
    // in the target is a direct parent
    auto frame_id = source_id;
    TimeAndFrameID tmp;
    uint32_t depth = 0;
    auto common_time = ros::TIME_MAX;
    while (frame_id != 0){
      auto cache = getFrame(frame_id);
      if(!cache){
        // There will be no cache for the very root of the tree
        // if frame id is not 0 and not parent, then there should be cache.
        break;
      }

      un_locker.rLockIfItWasNot(frame_id);
      auto latest = cache->getLatestTimeAndParent();
      if(latest.second == 0){
        // Just break out here... there may still be a path from source -> target
        break;
      }

      if(!latest.first.isZero()){
        common_time = std::min(latest.first, common_time);
      }

      lookup_cache.push_back(latest);

      frame_id = latest.second; // going up

      if(frame_id == target_id){ // s (->)(+) t
        out_time = common_time;
        if(out_time == ros::TIME_MAX){
          out_time = ros::Time();
        }
        return tf2_msgs::NO_ERROR;
      }

      ++depth;
      if(depth > MAX_GRAPH_DEPTH){
        if(err_str){
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains loop" << std::endl;
          *err_str = ss.str();
        }
        return tf2_msgs::LOOKUP_ERROR;
      }
    }

    // reach to the root
    // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
    frame_id = target_id;
    depth = 0;
    common_time = ros::TIME_MAX;
    tf2::CompactFrameID common_parent = 0;
    for(;;){
      auto cache = getFrame(frame_id);
      if(!cache){
        // should be reach to root.
        break;
      }
      un_locker.rLockIfItWasNot(frame_id);
      auto latest = cache->getLatestTimeAndParent();
      if(latest.second == 0){
        break;
      }
      if(!latest.first.isZero()){
        common_time = std::min(latest.first, common_time);
      }

      auto it = std::find_if(lookup_cache.begin(), lookup_cache.end(), TimeAndFrameIDComparator(latest.second));
      if(it != lookup_cache.end()){
        common_parent = it->second;
        break;
      }

      frame_id = latest.second;
      if(frame_id == source_id){ // t (->)(+) s
        out_time = common_time;
        if(out_time == ros::TIME_MAX){
          out_time = ros::Time();
        }
        return tf2_msgs::NO_ERROR;
      }

      ++depth;
      if(depth > MAX_GRAPH_DEPTH){
        if(err_str){
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains loop" << std::endl;
          *err_str = ss.str();
        }
        return tf2_msgs::LOOKUP_ERROR;
      }
    }

    if(common_parent == 0){ // no common parent found!
      createConnectivityErrorString(source_id, target_id, err_str);
      return tf2_msgs::CONNECTIVITY_ERROR;
    }

    // Loop through the source -> root list until we hit the common parent
    {
      auto it = lookup_cache.begin();
      auto end = lookup_cache.end();
      for(; it != end; ++it){
        if(!it->first.isZero()){
          common_time = std::min(common_time, it->first);
        }

        if(it->second == common_parent){
          break;
        }
      }
    }

    if(common_time == ros::TIME_MAX){
      common_time = ros::Time();
    }

    out_time = common_time;
    return tf2_msgs::NO_ERROR;
  }

  tf2_msgs::TF2Error walkToTopParent(
    TransformAccum &f, ros::Time time, tf2::CompactFrameID target_id,
    tf2::CompactFrameID source_id, std::string *err_str,
    std::vector<tf2::CompactFrameID> *frame_chain,
    ScopedReadUnLocker &un_locker) const{
    // assert whole tree is r-locked, and scoped read unlock has created.

    if(frame_chain){
      frame_chain->clear();
    }

    // Short circuit1
    if(source_id == target_id){
      f.finalize(Identity, time);
      return tf2_msgs::NO_ERROR;
    }

    // get latest time through path
    if(time == ros::Time()){
      // what to do?
      // one simple idea is just locking up the tree at here.
      // another way is to do accum when looking and locking up the tree, so this doubles speed of calculation.
      // however, for simplicity, we just do early lock at here.
      auto ret = getLatestCommonTime(target_id, source_id, un_locker, time, err_str);
      if(ret != tf2_msgs::NO_ERROR){
        return ret;
      }
    }

    // Phase1. walk the tree from source to root.
    tf2::CompactFrameID frame_id = source_id;
    tf2::CompactFrameID top_parent = frame_id;
    uint32_t depth = 0;
    std::string extrapolation_err_str;
    bool extrapolation_might_have_occurred = false;

    while (frame_id != 0){
      auto cache = getFrame(frame_id);
      if(frame_chain)
        frame_chain->push_back(frame_id);

      if(!cache){
        top_parent = frame_id;
        break;
      }

      un_locker.rLockIfItWasNot(frame_id);

      auto parent = f.gather(cache, time, err_str ? &extrapolation_err_str : nullptr);
      if(parent == 0){
        // s -> .. -> frame_id -> (out of time (extrapolation))
        // there may still be t -> s, so just break;
        top_parent = frame_id;
        extrapolation_might_have_occurred = true;
        break;
      }

      // s -> .. -> t
      // Early out
      if(frame_id == target_id){
        f.finalize(TargetParentOfSource, time);
        return tf2_msgs::NO_ERROR;
      }
      f.accum(true);

      frame_id = parent;
      ++depth;
      if(depth > MAX_GRAPH_DEPTH){
        if(err_str){
          std::stringstream ss;
          ss << "The tf tree is invalid because it contains a loop." << std::endl;
          *err_str = ss.str();
        }

        return tf2_msgs::LOOKUP_ERROR;
      }
    }

    // s --> (extrapolation) or s --> t or s --> INF

    // Now walk from target to top until top_parent
    frame_id = target_id;
    depth = 0;
    std::vector<tf2::CompactFrameID> reverse_frame_chain;
    while (frame_id != top_parent){
      auto cache = getFrame(frame_id);
      if(frame_chain){
        reverse_frame_chain.push_back(frame_id);
      }
      if(!cache){
        break;
      }

      un_locker.rLockIfItWasNot(frame_id);

      auto parent = f.gather(cache, time, err_str);
      if(parent == 0){
        if(err_str){
          char str[1000];
          snprintf(str, sizeof(str), "when looking up transform from frame");
          *err_str = str;
        }

        return tf2_msgs::EXTRAPOLATION_ERROR;
      }

      // Early out, t -> .. -> s
      if(frame_id == source_id){
        f.finalize(SourceParentOfTarget, time);
        if(frame_chain){
          frame_chain->swap(reverse_frame_chain);
          std::reverse(frame_chain->begin(), frame_chain->end());
        }
        return tf2_msgs::NO_ERROR;
      }

      f.accum(false);
      frame_id = parent;

     ++depth;
     if(depth > MAX_GRAPH_DEPTH){
       if(err_str){
         std::stringstream ss;
         ss << "The tf tree is invalid because"
               " it contains a loop." << std::endl;
         *err_str = ss.str();
       }
       return tf2_msgs::LOOKUP_ERROR;
     }
    }

    if(frame_id != top_parent){
      if(extrapolation_might_have_occurred){
        if(err_str){
          char str[1000];
          snprintf(str, sizeof(str), "when looking up transform from frame");
          *err_str = str;
        }

        return tf2_msgs::EXTRAPOLATION_ERROR;
      }

      createConnectivityErrorString(source_id, target_id, err_str);
      return tf2_msgs::CONNECTIVITY_ERROR;
    }else if(frame_chain){
      reverse_frame_chain.push_back(frame_id);
    }

    f.finalize(FullPath, time);
    if(frame_chain){
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

    return tf2_msgs::NO_ERROR;
  }

public:
  // under here, everyone can access to each method, so
  // be sure locking is working properly.

  geometry_msgs::TransformStamped lookupTransform(
      const std::string &target_frame,
      const std::string &source_frame,
      const ros::Time &time
    )const{
    std::set<tf2::CompactFrameID> r_locked_set{};
    ScopedReadUnLocker un_locker(r_locked_set, frameIdMutex);
    ScopedTreeReadUnLocker tree_un_locker(treeMutex);
    tree_un_locker.rLock();

    // short circuit
    if(target_frame == source_frame){
      geometry_msgs::TransformStamped identity;
      identity.header.frame_id = target_frame;
      identity.child_frame_id = source_frame;
      identity.transform.rotation.w = 1;

      if(time == ros::Time()){
        auto target_id = lookupFrameNumber(target_frame);
        auto cache = getFrame(target_id);
        if(cache){
          un_locker.rLockIfItWasNot(target_id);
          identity.header.stamp = cache->getLatestTimestamp();
        }else{
          identity.header.stamp = time;
        }
      }else{
        identity.header.stamp = time;
      }

      return identity;
    }

    auto target_id = validateAndGetFrameId("lookupTransform arg target_frame", target_frame);
    auto source_id = validateAndGetFrameId("lookupTransform arg source_frame", source_frame);
    assert(target_id != source_id);

    std::string err_str;
    TransformAccum accum;
    int ret = walkToTopParent(accum, time, target_id, source_id, &err_str, nullptr, un_locker);
    if(ret != tf2_msgs::NO_ERROR){
      switch (ret) {
        case tf2_msgs::CONNECTIVITY_ERROR:
          throw tf2::ConnectivityException(err_str);
        case tf2_msgs::EXTRAPOLATION_ERROR:
          throw tf2::ExtrapolationException(err_str);
        case tf2_msgs::LOOKUP_ERROR:
          throw tf2::LookupException(err_str);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", ret);
          assert(false);
      }
    }

    geometry_msgs::TransformStamped output_trans;
    transformTF2ToMsg(accum.resultQuat,
                      accum.resultVec,
                      output_trans,
                      accum.time,
                      target_frame, source_frame);
    return output_trans;

  }

  bool setTransforms(
    const std::vector<geometry_msgs::TransformStamped> & transforms,
    const std::string& authority, bool is_static){

    std::vector<geometry_msgs::TransformStamped> stripped{};
    for(auto &e: transforms){
      geometry_msgs::TransformStamped tmp = e;
      tmp.header.frame_id = stripSlash(tmp.header.frame_id);
      tmp.child_frame_id = stripSlash(tmp.child_frame_id);
      stripped.push_back(e);
    }

    bool err_exists = false;
    for(auto &e: stripped){
      if (e.child_frame_id == e.header.frame_id)
      {
        CONSOLE_BRIDGE_logError("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), e.child_frame_id.c_str());
        err_exists = true;
      }

      if (e.child_frame_id == "")
      {
        CONSOLE_BRIDGE_logError("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
        err_exists = true;
      }

      if (e.header.frame_id == "")
      {
        CONSOLE_BRIDGE_logError("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", e.child_frame_id.c_str(), authority.c_str());
        err_exists = true;
      }

      if (std::isnan(e.transform.translation.x)
      ||  std::isnan(e.transform.translation.y)
      ||  std::isnan(e.transform.translation.z)
      ||  std::isnan(e.transform.rotation.x)
      ||  std::isnan(e.transform.rotation.y)
      ||  std::isnan(e.transform.rotation.z)
      ||  std::isnan(e.transform.rotation.w))
      {
        CONSOLE_BRIDGE_logError("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
                                e.child_frame_id.c_str(), authority.c_str(),
                                e.transform.translation.x, e.transform.translation.y, e.transform.translation.z,
                                e.transform.rotation.x, e.transform.rotation.y, e.transform.rotation.z, e.transform.rotation.w
        );
        err_exists = true;
      }

      bool valid = std::abs((e.transform.rotation.w * e.transform.rotation.w
                             + e.transform.rotation.x * e.transform.rotation.x
                             + e.transform.rotation.y * e.transform.rotation.y
                             + e.transform.rotation.z * e.transform.rotation.z) - 1.0f) < QUATERNION_NORMALIZATION_TOLERANCE;

      if (!valid)
      {
        CONSOLE_BRIDGE_logError("TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of an invalid quaternion in the transform (%f %f %f %f)",
                                e.child_frame_id.c_str(), authority.c_str(),
                                e.transform.rotation.x, e.transform.rotation.y, e.transform.rotation.z, e.transform.rotation.w);
        err_exists = true;
      }
    }

    if(err_exists){
      return false;
    }



    std::set<tf2::CompactFrameID> lock_list{};
    ScopedWriteUnLocker un_locker(lock_list, frameIdMutex);
    treeMutex.r_lock(); // This tree mutex can be upgraded to write lock.
    bool lock_upgraded = false;

    for(auto &e: stripped){
      auto id = lookupOrInsertFrameNumber(e.child_frame_id, lock_upgraded);
      auto frame = getFrame(id);
      if(frame == nullptr){
        frame = allocateFrame(id, is_static);
      }

      assert(lock_list.find(id) == lock_list.end());
      frameIdMutex[id]->w_lock();
      lock_list.insert(id);

      std::string err_str;
      if(frame->insertData(tf2::TransformStorage(e, lookupOrInsertFrameNumber(e.header.frame_id, lock_upgraded), id), &err_str)){
        // set frame authority
      }else{
        CONSOLE_BRIDGE_logWarn((err_str+" for frame %s at time %lf according to authority %s").c_str(), e.child_frame_id.c_str(), e.header.stamp.toSec(), authority.c_str());
      }
    }

    if(lock_upgraded){
      treeMutex.w_unlock();
    }else{
      treeMutex.r_unlock();
    }

    return true;
  }

  bool setTransform(
    geometry_msgs::TransformStamped transform,
    const std::string &authority, bool is_static = false
    ){
    std::vector<geometry_msgs::TransformStamped> vec{transform};
    setTransforms(vec, authority, is_static);
  }

  // each item is protected by frame id mutex
  std::vector<TimeCacheInterfacePtr> frames;
  mutable std::vector<RWLockPtr> frameIdMutex{};
  // when insert and delete happen, these items are modified
  // At here, we only care about w-r, so WE DONT PROTECT.
  typedef std::unordered_map<std::string, tf2::CompactFrameID> StringToCompactFrameID;
  StringToCompactFrameID frameToId;
  std::vector<std::string> idToFrame;
  mutable RWLock treeMutex{};


  ros::Duration cacheTime;

};

}

#endif //ALT_TF_ALT_BUFFER_CORE_H
