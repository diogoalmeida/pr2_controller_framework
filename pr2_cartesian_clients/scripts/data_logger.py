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
init_log_time = rospy.Time()
max_log_duration = rospy.Duration(-1)
log_directory_name = " "
sub_type = None
subscriber = None

def feedbackCallback(feedback_msg):
    """Data that will be monitored."""
    global subscription_topic_name
    global bag

    if is_logging:
        if rospy.Time.now() - init_log_time < max_log_duration:
            try:
                bag.write(subscription_topic_name, feedback_msg)
            except:
                print "Unexpected error:", sys.exc_info()[0]


def loggingCallback(logging_req):
    """Toggle logging data into the bag file."""
    global is_logging
    global bag
    global init_log_time
    global max_log_duration
    global sub_type
    global subscriber

    rospack = rospkg.RosPack()

    if logging_req.log_type == logging_req.START_LOGGING:
        bag_dir = rospack.get_path("pr2_cartesian_clients") + "/data/" + log_directory_name
        print "bag dir: " + bag_dir

        if not os.path.exists(bag_dir):
            print "making dir: " + bag_dir
            os.makedirs(bag_dir)

        is_logging = True
        subscriber = rospy.Subscriber(subscription_topic_name, sub_type, feedbackCallback)
        bag = rosbag.Bag(bag_dir + "/" + logging_req.name + ".bag", 'w')
        init_log_time = rospy.Time.now()
        max_log_duration = rospy.Duration(logging_req.max_record_time)
        rospy.loginfo("Data logger starting")
        return True
    elif logging_req.log_type == logging_req.SAVE_BAG:
        is_logging = False
        rospy.loginfo("Data logger stopping. Saving bag")
        bag.close()
        subscriber.unregister()
        bag = None
        return True
    else:
        is_logging = False
        rospy.loginfo("Data logger stopping. Discarding bag")
        path = bag.filename
        bag.close()
        subscriber.unregister()
        bag = None
        os.remove(path)
        return True


def loadParams():
    """Get parameters from the parameter server."""
    global subscription_topic_name
    global toggle_logging_service_name
    global log_directory_name

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

    if rospy.has_param('/manipulation_client/experiment/logging/log_directory'):
        log_directory_name = rospy.get_param("/manipulation_client/experiment/logging/log_directory")
    else:
        rospy.logerr("Missing toggle logging service name (experiment/logging/log_directory)")
        return False

    return True

if __name__ == '__main__':
    rospy.init_node('data_logger')
    global sub_type

    if loadParams():
        logging_service = rospy.Service(toggle_logging_service_name, LogMessages, loggingCallback)

        if len(sys.argv) > 1:
            if sys.argv[1] == "debug":
                sub_type = pr2_algorithms.msg.TestBedFeedback
            else:
                sub_type = pr2_cartesian_controllers.msg.ManipulationControllerActionFeedback
        else:
            sub_type = pr2_cartesian_controllers.msg.ManipulationControllerActionFeedback

        rospy.spin()
