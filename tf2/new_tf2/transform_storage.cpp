#include "transform_storage.h"
#include <geometry_msgs/TransformStamped.h>

namespace new_tf2{
  TransformStorage::TransformStorage() {}
  TransformStorage::TransformStorage(
    const geometry_msgs::TransformStamped &data,
    TimeCache* parent_)
  :  stamp(data.header.stamp), parent(parent_)
   {
    const geometry_msgs::Quaternion &o = data.transform.rotation;
    rotation = tf2::Quaternion(o.x, o.y, o.z, o.w);
    const geometry_msgs::Vector3 &v = data.transform.translation;
    vec[0] = v.x;
    vec[1] = v.y;
    vec[2] = v.z;
  }
}