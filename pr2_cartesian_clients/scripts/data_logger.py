"""Provide a node that will log data from the given topics until a stop is requested."""
import rospy
import rosbag
from std_msgs.msg import Bool
import pr2_cartesian_controllers.msg

is_logging = False


def loggingCallback(logging_msg):
    """Toggle logging data into the bag file."""
    global is_logging

    if logging_msg.data:
        is_logging = True
        rospy.loginfo("Data logger starting")
    else:
        is_logging = False
        rospy.loginfo("Data logger stopping")


if __name__ == '__main__':
    rospy.init_node('data_logger')

    rospy.Subscriber("/data_logger", Bool, loggingCallback)
