#include "silo_buffer_core.h"
#include "tf2/time_cache.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/TF2Error.h"

#include <cassert>
#include <console_bridge/console.h>
#include "tf2/LinearMath/Transform.h"
#include <boost/foreach.hpp>
#include <atomic>
#include <thread>
#include <chrono>

using std::chrono::operator""ms;
using tf2::CompactFrameID;

namespace silo_tf2
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

  bool SiloBufferCore::warnFrameId(const char* function_name_arg, const std::string& frame_id) const noexcept
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

  tf2::CompactFrameID SiloBufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
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

    tf2::CompactFrameID id = lookupFrameNumber(frame_id);
    if (id == 0)
    {
      std::stringstream ss;
      ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
      throw tf2::LookupException(ss.str());
    }

    return id;
  }

  SiloBufferCore::SiloBufferCore(ros::Duration cache_time)
    : cache_time_(cache_time)
    , using_dedicated_thread_(false)
  {
    frameIDs_["NO_PARENT"] = 0;
    frames_.reserve(10005);
    frames_.push_back(TimeCacheInterfacePtr());
    frameIDs_reverse.emplace_back("NO_PARENT");
//  frame_each_mutex_.emplace_back(std::make_shared<RWLock>());

    frame_each_mutex_ = new std::array<VRWLock, 1'000'005>();
    CONSOLE_BRIDGE_logWarn("Now using ALT TF!!!");
  }

  SiloBufferCore::~SiloBufferCore()
  {
    delete frame_each_mutex_;
    for(auto e : frames_){
      delete e;
    }
  }

// thread safe
  bool SiloBufferCore::setTransform(
    const geometry_msgs::TransformStamped& transform,
    const std::string &authority, bool is_static) noexcept{
    std::vector<geometry_msgs::TransformStamped> vec{transform};
    setTransforms(vec, authority, is_static);
  }

// thread safe
  bool SiloBufferCore::setTransforms(
    const std::vector<geometry_msgs::TransformStamped>& transforms,
    const std::string& authority, bool is_static, WriteStat *result) noexcept{
    std::vector<geometry_msgs::TransformStamped> stripped{};
    for(auto &e: transforms){
      geometry_msgs::TransformStamped tmp = e;
      tmp.header.frame_id = stripSlash(tmp.header.frame_id);
      tmp.child_frame_id = stripSlash(tmp.child_frame_id);
      stripped.push_back(e);
    }

    bool error_exists = false;
    for(auto &e: stripped){
      if (e.child_frame_id == e.header.frame_id)
      {
        CONSOLE_BRIDGE_logError("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), e.child_frame_id.c_str());
        error_exists = true;
      }

      if (e.child_frame_id.empty())
      {
        CONSOLE_BRIDGE_logError("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
        error_exists = true;
      }

      if (e.header.frame_id.empty())
      {
        CONSOLE_BRIDGE_logError("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", e.child_frame_id.c_str(), authority.c_str());
        error_exists = true;
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
        error_exists = true;
      }

      bool valid = std::abs((e.transform.rotation.w * e.transform.rotation.w
                             + e.transform.rotation.x * e.transform.rotation.x
                             + e.transform.rotation.y * e.transform.rotation.y
                             + e.transform.rotation.z * e.transform.rotation.z) - 1.0f) < QUATERNION_NORMALIZATION_TOLERANCE;

      if (!valid){
        CONSOLE_BRIDGE_logError("TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of an invalid quaternion in the transform (%f %f %f %f)",
                                e.child_frame_id.c_str(), authority.c_str(),
                                e.transform.rotation.x, e.transform.rotation.y, e.transform.rotation.z, e.transform.rotation.w);
        error_exists = true;
      }
    }

    if (error_exists){
      return false;
    }

    // before testTransformableRequests, you have to unlock.
    {
      std::vector<std::tuple<TimeCacheInterfacePtr, geometry_msgs::TransformStamped, tf2::CompactFrameID>> write_set{};

      phase_1:
      for(auto &e: stripped){ // assert stripped is sorted.
        auto id = lookupOrInsertFrameNumber(e.child_frame_id);
        auto frame = getFrame(id);
        if(frame == nullptr){
          frame = allocateFrame(id, is_static);
        }

        frame_each_mutex_->at(id).wLock();
        write_set.emplace_back(frame, e, id);
      }

      phase_2:
      // no read, so just skip.
      phase_3:
      for(auto &w: write_set){
        auto frame = std::get<0>(w);
        auto e = std::get<1>(w);
        auto id = std::get<2>(w);

        std::string err_str;
        auto write = tf2::TransformStorage(e, lookupOrInsertFrameNumber(e.header.frame_id), id);
        *frame = write;
        frame_authority_[id] = authority;
        frame_each_mutex_->at(id).wUnLock();
      }
    }
    // set Transform and test req aren't handled serialized fashion!
    //testTransformableRequests();
    return true;
  }

// thread safe
  TimeCacheInterfacePtr SiloBufferCore::allocateFrame(tf2::CompactFrameID cfid, bool is_static) noexcept
  {
//    TimeCacheInterfacePtr frame_ptr = frames_[cfid];
//    if (is_static) {
//      frames_[cfid] = TimeCacheInterfacePtr(new tf2::StaticCache());
//    } else {
//      frames_[cfid] = TimeCacheInterfacePtr(new tf2::TimeCache(cache_time_));
//    }
    frames_[cfid] = new tf2::TransformStorage();

    return frames_[cfid];
  }

  enum WalkEnding
  {
    Identity,
    TargetParentOfSource,
    SourceParentOfTarget,
    FullPath,
  };

  template<typename F>
  int SiloBufferCore::walkToTopParentLatest(
    F& f, CompactFrameID target_id,CompactFrameID source_id,
    std::string* error_string,
    ReadChecker &read_checker,
    ReadStat *stat) const noexcept
  {
retry:

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

      read_checker.addRLock(frame);

      if(stat != nullptr){
        stat->timestamps.push_back(cache->stamp_.toNSec());
      }

      CompactFrameID parent = f.gather(cache, ros::Time(0), &extrapolation_error_string);
      if (parent == 0)
      {
        // s -> .. -> frame -> (out of time (extrapolation))
        // Just break out here... there may still be a path from source -> target
        top_parent = frame;
        extrapolation_might_have_occurred = true;
        break;
      }

      // s -> .. -> t
      // Early out... target frame is a direct parent of the source frame
      if (frame == target_id)
      {
        if(!read_checker.check()){
          stat->abortCount++;
          read_checker.clear();
          stat->timestamps.clear();
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
          ss << "The tf tree is invalid because it contains a loop." << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
      }
    }

    // These checks have done:
    // s --> (extrapolation)
    // s --> t
    // s --> ...(INF loop)

    // Now walk to the top parent from the target frame, accumulating its transform
    frame = target_id;
    depth = 0;

    while (frame != top_parent)
    {
      TimeCacheInterfacePtr cache = getFrame(frame);
      if (!cache){
        break;
      }

      read_checker.addRLock(frame);
      CompactFrameID parent = f.gather(cache, ros::Time(0), error_string);
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
        if(!read_checker.check()){
          stat->abortCount++;
          read_checker.clear();
          stat->timestamps.clear();
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
          ss << "The tf tree is invalid because it contains a loop." << std::endl;
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

    if(!read_checker.check()){
      stat->abortCount++;
      read_checker.clear();
      stat->timestamps.clear();
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
    {
    }

    CompactFrameID gather(TimeCacheInterfacePtr cache, ros::Time time, std::string* error_string, ReadStat *stat = nullptr)
    {
      st = *cache;
      return st.frame_id_;
    }

    void accum(bool source)
    {
      if (source)
      {
        source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
        source_to_top_quat = st.rotation_ * source_to_top_quat;
      }
      else
      {
        target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
        target_to_top_quat = st.rotation_ * target_to_top_quat;
      }
    }

    void finalize(WalkEnding end, ros::Time _time)
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

    tf2::TransformStorage st;
    ros::Time time;
    tf2::Quaternion source_to_top_quat;
    tf2::Vector3 source_to_top_vec;
    tf2::Quaternion target_to_top_quat;
    tf2::Vector3 target_to_top_vec;

    tf2::Quaternion result_quat;
    tf2::Vector3 result_vec;
  };

// thread safe, but throws.
  geometry_msgs::TransformStamped SiloBufferCore::lookupLatestTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    ReadStat *stat) const noexcept(false)
  {
    ReadChecker read_checker(*frame_each_mutex_);

    if (target_frame == source_frame) {
      geometry_msgs::TransformStamped identity;
      identity.header.frame_id = target_frame;
      identity.child_frame_id = source_frame;
      identity.transform.rotation.w = 1;

      CompactFrameID target_id = lookupFrameNumber(target_frame);
      TimeCacheInterfacePtr cache = getFrame(target_id);
      if(cache){
        read_checker.addRLock(target_id);
        identity.header.stamp = cache->stamp_;
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
    int retval = walkToTopParentLatest(accum, target_id, source_id, &error_string, read_checker, stat);
    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
        case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw tf2::ConnectivityException(error_string);
        case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw tf2::ExtrapolationException(error_string);
        case tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw tf2::LookupException(error_string);
        default:
          CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
          assert(0);
      }
    }

    geometry_msgs::TransformStamped output_transform;
    transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform, accum.time, target_frame, source_frame);
    return output_transform;
  }

  TimeCacheInterfacePtr SiloBufferCore::getFrame(CompactFrameID frame_id) const noexcept
  {
    if (frame_id >= frames_.size())
      return nullptr;
    else
    {
      return frames_[frame_id];
    }
  }

// not thread safe?
// may be it is just safe because delete don't occur.
  CompactFrameID SiloBufferCore::lookupFrameNumber(const std::string& frameid_str) const noexcept
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

// thread safe??
  CompactFrameID SiloBufferCore::lookupOrInsertFrameNumber(
    const std::string& frameid_str) noexcept{
    CompactFrameID retval = 0;
    auto map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end()){
      retval = CompactFrameID(frames_.size());

      // TODO: is this really safe? what happen when push_back causes different timing??
      frames_.emplace_back();//Just a place holder for iteration
//    frame_each_mutex_.emplace_back(std::make_unique<RWLock>());
      frameIDs_[frameid_str] = retval;
      frameIDs_reverse.push_back(frameid_str);
    }
    else
      retval = frameIDs_[frameid_str];

    return retval;
  }

  const std::string& SiloBufferCore::lookupFrameString(CompactFrameID frame_id_num) const noexcept(false)
  {
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];
  }

  void SiloBufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const noexcept(false)
  {
    if (!out)
    {
      return;
    }
    *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                       lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                       "Tf has two or more unconnected trees.");
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
} // namespace tf2
