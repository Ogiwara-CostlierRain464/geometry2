#!/usr/bin/env python
import tf
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import rospy

if __name__ == "__main__":
    rospy.init_node("publisher", anonymous=True)
    # publishes every 150Hz.
    br = tf.TransformBroadcaster()
    r = rospy.Rate(150)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "a", "b")
        r.sleep()
