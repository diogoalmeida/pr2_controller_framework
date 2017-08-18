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
log_type = " "

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
    global log_type

    if rospy.has_param('logging_topic'):
        subscription_topic_name = rospy.get_param("logging_topic")
    else:
        rospy.logerr("Missing logging topic name (logging_topic)")
        return False

    if rospy.has_param('toggle_logging_service'):
        toggle_logging_service_name = rospy.get_param("toggle_logging_service")
    else:
        rospy.logerr("Missing toggle logging service name (toggle_logging_service)")
        return False

    if rospy.has_param('log_directory'):
        log_directory_name = rospy.get_param("log_directory")
    else:
        rospy.logerr("Missing toggle logging service name (log_directory)")
        return False

    if rospy.has_param('log_type'):
        log_type = rospy.get_param("log_type")
    else:
        rospy.logerr("Missing toggle logging service name (log_type)")
        return False

    return True

if __name__ == '__main__':
    rospy.init_node('data_logger')
    global sub_type
    global log_type

    if loadParams():
        logging_service = rospy.Service(toggle_logging_service_name, LogMessages, loggingCallback)

        if log_type == "debug":
            sub_type = pr2_algorithms.msg.TestBedFeedback
        elif log_type == "mechanism":
            sub_type = pr2_cartesian_controllers.msg.ManipulationControllerActionFeedback
        elif log_type == "folding":
            sub_type = pr2_cartesian_controllers.msg.FoldingControllerActionFeedback
        elif log_type == "mechanism":
            sub_type = pr2_cartesian_controllers.msg.MechanismControllerActionFeedback
        else:
            rospy.logerr("unknwon log type: " + log_type)
            break

        rospy.spin()
