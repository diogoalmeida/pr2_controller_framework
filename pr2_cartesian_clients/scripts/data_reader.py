#!/usr/bin/env python
"""Provide a node that will read data from logged bags"""
from __future__ import (absolute_import, division, print_function,
                        unicode_literals)
import rospy
import rosbag
import rospkg
import glob
import numpy as np
import matplotlib.pyplot as plt


def loadParams():
    """Get parameters from the parameter server."""
    return True


def getDir():
    """Get bag dir."""
    rospack = rospkg.RosPack()
    return rospack.get_path("pr2_cartesian_clients") + "/data/results/"


def openBag(name):
    """Open bag with given name in data/results."""
    bag = rosbag.Bag(getDir() + name + ".bag", 'r')
    return bag


if __name__ == '__main__':
    rospy.init_node('data_reader')
    gray = (0.85, 0.87, 0.89)
    bag_prefix = "Manipulation_Experiment_"
    num = 0

    for num in range(len(glob.glob(getDir() + "*.bag"))):
        t = np.array([])
        x_c_hat = np.array([])
        x_d = np.array([])
        var_x = np.array([])
        theta_c_hat = np.array([])
        theta_d = np.array([])
        var_theta = np.array([])
        f_c = np.array([])
        f_c_hat = np.array([])
        var_f = np.array([])
        f_d = np.array([])

        if loadParams():
            bag_name = bag_prefix + str(num)
            bag = openBag(bag_name)

            for topic, msg, time in bag.read_messages():
                t = np.append(t, [time.to_sec()])
                x_c_hat = np.append(x_c_hat, [msg.feedback.x_c_hat])
                x_d = np.append(x_d, [msg.feedback.x_d])
                var_x = np.append(var_x, [msg.feedback.var_x])
                theta_c_hat = np.append(theta_c_hat, [msg.feedback.theta_c_hat])
                theta_d = np.append(theta_d, [msg.feedback.theta_d])
                var_theta = np.append(var_theta, [msg.feedback.var_theta])
                f_c = np.append(f_c, [msg.feedback.f_c])
                f_c_hat = np.append(f_c_hat, [msg.feedback.f_c_hat])
                f_d = np.append(f_d, [msg.feedback.f_d])
                var_f = np.append(var_f, [msg.feedback.var_f])

            # Print one set of results
            t = t - t[0]

            plt.figure(1)
            plt.subplot(311)
            plt.plot(t, x_d - x_c_hat, color=gray)
            plt.title('X error')
            plt.legend(['$x_d - \hat{x}_c$'])
            plt.grid(True)

            plt.subplot(312)
            plt.plot(t, theta_d - theta_c_hat, color=gray)
            plt.title('Theta error')
            plt.legend(['$\\theta_d - \hat{\\theta}_c$'])
            plt.grid(True)

            plt.subplot(313)
            plt.plot(t, f_c_hat - f_d, color=gray)
            plt.plot(t, f_c - f_d, color=gray)
            plt.title('Force error')
            plt.legend(['$f_d - \hat{f}_c$', '$f_d - f_c$'])
            plt.xlabel('Time [s]')
            plt.grid(True)
    plt.show()
