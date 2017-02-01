#!/usr/bin/env python
"""Provide a node that will read data from logged bags"""
import rospy
import rosbag
import rospkg
import numpy as np
import matplotlib.pyplot as plt


def loadParams():
    """Get parameters from the parameter server."""
    return True


def openBag(name):
    """Open bag with given name in data/results."""
    rospack = rospkg.RosPack()

    bag = rosbag.Bag(rospack.get_path("pr2_cartesian_clients") + "/data/results/" + name + ".bag", 'r')
    return bag

if __name__ == '__main__':
    rospy.init_node('data_reader')

    t = np.array([])
    x_c_hat = np.array([])
    x_d = np.array([])
    theta_c_hat = np.array([])
    theta_d = np.array([])
    f_c = np.array([])
    f_c_hat = np.array([])
    f_d = np.array([])

    if loadParams():
        bag_name = "Manipulation_Experiment_0"
        bag = openBag(bag_name)

        for topic, msg, time in bag.read_messages():
            t = np.append(t, [time.to_sec()])
            x_c_hat = np.append(x_c_hat, [msg.feedback.x_c_hat])
            x_d = np.append(x_d, [msg.feedback.x_d])
            theta_c_hat = np.append(theta_c_hat, [msg.feedback.theta_c_hat])
            theta_d = np.append(theta_d, [msg.feedback.theta_d])
            f_c = np.append(f_c, [msg.feedback.f_c])
            f_c_hat = np.append(f_c_hat, [msg.feedback.f_c_hat])
            f_d = np.append(f_d, [msg.feedback.x_c_hat])

        t = t - t[0]
        plt.plot(t, np.absolute(theta_c_hat - theta_d))
        plt.show()
