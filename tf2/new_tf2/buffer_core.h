#ifndef NEW_BUFFER_CORE_H
#define NEW_BUFFER_CORE_H


#include <cinttypes>
#include <string>
#include <ros/duration.h>
#include "geometry_msgs/TransformStamped.h"
#include "../include/tf2/stat.h"
#include "time_cache.h"
#include "tf2_msgs/TF2Error.h"

#include <tbb/concurrent_unordered_map.h>

namespace new_tf2{
  struct TransformAccum;
  class BufferCore{
  public:
    static const uint64_t MAX_GRAPH_DEPTH = 1'000'000;

    explicit BufferCore();

    bool setTransform(const geometry_msgs::TransformStamped &transform,
                      const std::string &auth) noexcept;

    bool setTransformsXact(const std::vector<geometry_msgs::TransformStamped> &transforms,
                           const std::string &authority,
                           tf2::WriteStat *stat = nullptr);

    geometry_msgs::TransformStamped
    lookupLatestTransformXact(const std::string& target_frame,
                    const std::string& source_frame,
                    tf2::ReadStat *stat = nullptr) const noexcept(false);


  private:
    tbb::concurrent_unordered_map<std::string, TimeCache*> frames;

    TimeCache* getOrInsertTimeCache(const std::string& frame, tf2::WriteStat *stat = nullptr);
    TimeCache* validateFrame(const char* function_name_arg, const std::string& frame_id) const;
    TimeCache* getFrame(const std::string &frame) const;

    int walkToTopParentLatest(
      TransformAccum &f,
      TimeCache* target,
      TimeCache* source,
      tf2::ReadStat *stat
      ) const noexcept;
  };
}


#endif //NEW_BUFFER_CORE_H
