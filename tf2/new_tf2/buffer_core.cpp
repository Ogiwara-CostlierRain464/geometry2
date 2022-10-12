#include "buffer_core.h"
#include <console_bridge/console.h>

#include "../include/tf2/exceptions.h"

#include <thread>
#include <chrono>
using std::chrono::operator""ms;

namespace new_tf2{

  static double QUATERNION_NORMALIZATION_TOLERANCE = 10e-3;

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


  inline bool checkTransformValid(
    const geometry_msgs::TransformStamped &tr,
    const std::string& authority){
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




  inline geometry_msgs::TransformStamped stripTransform(const geometry_msgs::TransformStamped &tr){
    geometry_msgs::TransformStamped tmp = tr;
    tmp.header.frame_id = stripSlash(tmp.header.frame_id);
    tmp.child_frame_id = stripSlash(tmp.child_frame_id);
    return tmp;
  }

  enum WalkEnding
  {
    Identity,
    TargetParentOfSource,
    SourceParentOfTarget,
    FullPath,
  };

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

    inline TimeCache* gatherLatest(TimeCache* cache)
    {
      const tf2::Quaternion &rotation = cache->storage.rotation;
      st_rotation_.m_floats[0] = rotation.m_floats[0];
      st_rotation_.m_floats[1] = rotation.m_floats[1];
      st_rotation_.m_floats[2] = rotation.m_floats[2];
      st_rotation_.m_floats[3] = rotation.m_floats[3];
      double *vec = cache->storage.vec;
      st_translation_.m_floats[0] = vec[0];
      st_translation_.m_floats[1] = vec[1];
      st_translation_.m_floats[2] = vec[2];
      return cache->storage.parent;
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

  BufferCore::BufferCore() {}


  bool BufferCore::setTransform(const geometry_msgs::TransformStamped &transform,
                    const std::string &auth) noexcept{
    std::vector<geometry_msgs::TransformStamped> vec{transform};
    return setTransformsXact(vec, auth);
  }


  bool BufferCore::setTransformsXact(
    const std::vector<geometry_msgs::TransformStamped> &transforms,
    const std::string &authority,
    tf2::WriteStat *stat) {
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

    std::vector<std::tuple<
      TimeCache*, // child
      TimeCache*, // parent
      geometry_msgs::TransformStamped>> write_set{};

    ScopedUnLocker un_locker{};
    try_lock:
    for(auto &e: stripped){
      TimeCache* child = getOrInsertTimeCache(e.child_frame_id, stat);
      TimeCache* parent = getOrInsertTimeCache(e.header.frame_id, stat);

      if(un_locker.wLockedSize() == 0){
        while (!un_locker.tryWLockIfNot(&child->lock)){
          if(stat){
            stat->tryWriteCount++;
          }
        }
      }else{
        bool lock_suc = un_locker.tryWLockIfNot(&child->lock);
        if(!lock_suc){
          if(stat){
            stat->localAbortCount++;
          }
          un_locker.unlockAll();
          write_set.clear();
          std::this_thread::sleep_for(1ms);
          goto try_lock;
        }
      }

      write_set.emplace_back(child, parent, e);
    }

    // all lock acquired, so write
    for(auto &e: write_set){
      auto child = std::get<0>(e);
      auto parent = std::get<1>(e);
      auto cti = std::get<2>(e);

      child->storage = TransformStorage(cti, parent);
      child->authority = authority;
    }

    return true;
  }

  TimeCache* BufferCore::getOrInsertTimeCache(const std::string& frame, tf2::WriteStat *stat){
    auto cache_itr = frames.find(frame);
    if(cache_itr != frames.end()){
      return cache_itr->second;
    }
    // cache not exist, so insert new TimeCache
    auto tmp = new TimeCache();
    // Insert may fail due to concurrent threads.
    // If failed, retrieve again.
    auto pair = frames.emplace(frame, tmp);
    if(pair.second){
      // insert succeeded.
      return tmp;
    }
    // Insert failed, so get existing cache.
    if(stat)
      stat->insertFailCount++;

    delete tmp;
    auto again_itr = frames.find(frame);
    if(again_itr == frames.end()){
      // Unexpected behaviour.
      assert(false && "CC failed!");
    }
    return again_itr->second;
  }

  geometry_msgs::TransformStamped
  BufferCore::lookupLatestTransformXact(
    const std::string& target_frame,
    const std::string& source_frame,
    tf2::ReadStat *stat) const noexcept(false){
    if(target_frame == source_frame){
      assert(false && "Skip impl");
    }
    TimeCache *source = validateFrame("lookupLatest", source_frame);
    TimeCache *target = validateFrame("lookupLatest", target_frame);
    assert(source != nullptr);
    assert(target != nullptr);
    assert(source != target);
    TransformAccum accum;
    int ret = walkToTopParentLatest(accum, target, source, stat);
    if(ret != tf2_msgs::TF2Error::NO_ERROR){
      switch (ret) {
        default:
          assert(false);
      }
    }

    geometry_msgs::TransformStamped out;
    transformTF2ToMsg(accum.result_quat,
                      accum.result_vec,
                      out,
                      accum.time,
                      target_frame,
                      source_frame);
    return out;
  }

  TimeCache*
  BufferCore::validateFrame(
    const char* function_name_arg,
    const std::string& frame_id) const{
    if(frame_id.empty()){
      throw tf2::InvalidArgumentException(function_name_arg);
    }
    if(startsWithSlash(frame_id)){
      throw tf2::InvalidArgumentException(function_name_arg);
    }
    auto cache = getFrame(frame_id);
    if(cache == nullptr){
      throw tf2::LookupException(function_name_arg);
    }else{
      return cache;
    }
  }

  TimeCache*
  BufferCore::getFrame(const std::string &frame) const{
    auto itr = frames.find(frame);
    if(itr == frames.end()){
      return nullptr;
    }else{
      return itr->second;
    }
  }


  int BufferCore::walkToTopParentLatest(
    TransformAccum &f,
    TimeCache* target,
    TimeCache* source,
    tf2::ReadStat *stat
  ) const noexcept{
    assert(target != nullptr);
    assert(source != nullptr);

    ScopedUnLocker un_locker{};
    if(target == source){
      f.finalize(Identity, ros::Time(0));
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    TimeCache* frame = source;
    TimeCache* top_parent = frame;
    uint64_t depth = 0;

    for (;;){
      un_locker.rLockIfNot(&frame->lock);

      if(stat){
        stat->timestamps.push_back(frame->storage.stamp.toNSec());
      }

      auto parent = f.gatherLatest(frame);

      if(parent == nullptr){
        top_parent = frame;
        break;
      }

      f.accum(true);

      if(parent == target){
        f.finalize(TargetParentOfSource, ros::Time(0));
        return tf2_msgs::TF2Error::NO_ERROR;
      }

      top_parent = frame;
      frame = parent;

      ++depth;
      if(depth > MAX_GRAPH_DEPTH){
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    frame = target;
    depth = 0;

    for(;;){
      un_locker.rLockIfNot(&frame->lock);

      if(stat)
        stat->timestamps.push_back(frame->storage.stamp.toNSec());

      auto parent = f.gatherLatest(frame);

      if(parent == nullptr){
        return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      }

      f.accum(false);

      if(parent == source){
        f.finalize(SourceParentOfTarget, ros::Time(0));
        return tf2_msgs::TF2Error::NO_ERROR;
      }
      if(parent == top_parent){
        break;
      }

      frame = parent;
      ++depth;
      if(depth > MAX_GRAPH_DEPTH){
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    if(frame != top_parent){
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    f.finalize(FullPath, ros::Time(0));

    return tf2_msgs::TF2Error::NO_ERROR;
  }
}