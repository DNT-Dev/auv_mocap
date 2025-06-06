#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def static_broadcaster():
    rospy.init_node('static_tf2_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "aruco_marker_0"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0  # identity quaternion

    broadcaster.sendTransform(static_transformStamped)
    rospy.loginfo("Broadcasting static transform from world -> aruco_marker_0 at origin")

    rospy.spin()

if __name__ == '__main__':
    static_broadcaster()

