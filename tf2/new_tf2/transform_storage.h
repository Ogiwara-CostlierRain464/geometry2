#ifndef NEW_TRANSFORM_STORAGE_H
#define NEW_TRANSFORM_STORAGE_H

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/types.h>
#include <ros/time.h>
#include <ros/message_forward.h>

namespace geometry_msgs
{
  ROS_DECLARE_MESSAGE(TransformStamped)
}

namespace new_tf2{
  class TimeCache;

  class alignas(128) TransformStorage{
  public:
    TransformStorage();
    explicit TransformStorage(const geometry_msgs::TransformStamped& data, TimeCache* parent);

    inline TransformStorage(const TransformStorage& rhs) = default;
    inline TransformStorage& operator=(const TransformStorage& rhs) = default;

    bool operator>(const TransformStorage& rhs) const
    {
      return stamp > rhs.stamp;
    }

    double vec[3]; // 24 byte
    tf2::Quaternion rotation; // 32byte
    TimeCache* parent = nullptr; // 8byte

    // -- 64 byte border --

    ros::Time stamp; // 8byte
    __attribute__((unused)) uint64_t _pad[6]; // 48byte, for padding
  };
}

#endif //NEW_TRANSFORM_STORAGE_H
