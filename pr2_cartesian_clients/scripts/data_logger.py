#!/usr/bin/env python
"""Provide a node that will log data from the given topics until a stop is requested."""
import os
import sys
import rospy
import rosbag
import rospkg
from pr2_cartesian_clients.srv import LogMessages
import pr2_cartesian_controllers.msg
import pr2_algorithms.msg

is_logging = False
subscription_topic_name = " "
toggle_logging_service_name = " "
bag = None


def feedbackCallback(feedback_msg):
    """Data that will be monitored."""
    global subscription_topic_name
    global bag

    if is_logging:
        try:
            bag.write(subscription_topic_name, feedback_msg)
        except:
            print "Unexpected error:", sys.exc_info()[0]


def loggingCallback(logging_req):
    """Toggle logging data into the bag file."""
    global is_logging
    global bag

    rospack = rospkg.RosPack()

    if logging_req.log_type == logging_req.START_LOGGING:
        is_logging = True
        bag = rosbag.Bag(rospack.get_path("pr2_cartesian_clients") + "/data/results/" + logging_req.name + ".bag", 'w')
        rospy.loginfo("Data logger starting")
        return True
    elif logging_req.log_type == logging_req.SAVE_BAG:
        is_logging = False
        rospy.loginfo("Data logger stopping. Saving bag")
        bag.close()
        bag = None
        return True
    else:
        is_logging = False
        rospy.loginfo("Data logger stopping. Discarding bag")
        path = bag.filename
        bag.close()
        bag = None
        os.remove(path)
        return True


def loadParams():
    """Get parameters from the parameter server."""
    global subscription_topic_name
    global toggle_logging_service_name

    if rospy.has_param('/manipulation_client/experiment/logging/logging_topic'):
        subscription_topic_name = rospy.get_param("/manipulation_client/experiment/logging/logging_topic")
    else:
        rospy.logerr("Missing logging topic name (experiment/logging/logging_topic)")
        return False

    if rospy.has_param('/manipulation_client/experiment/logging/toggle_logging_service'):
        toggle_logging_service_name = rospy.get_param("/manipulation_client/experiment/logging/toggle_logging_service")
    else:
        rospy.logerr("Missing toggle logging service name (experiment/logging/toggle_logging_service)")
        return False

    return True

if __name__ == '__main__':
    rospy.init_node('data_logger')

    if loadParams():
        logging_service = rospy.Service(toggle_logging_service_name, LogMessages, loggingCallback)
        rospy.Subscriber(subscription_topic_name, pr2_cartesian_controllers.msg.ManipulationControllerFeedback, feedbackCallback)
        # rospy.Subscriber(subscription_topic_name, pr2_algorithms.msg.TestBedFeedback, feedbackCallback)

        rospy.spin()
