import rospy
import tf2_msgs.msg


rospy.init_node("sync_camera")
rate = rospy.Rate(10)  # 10 Hz

