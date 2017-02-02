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

log_directory_name = " "
bag_prefix = "Thursday_02_02_afternoon_random_goals_"


def loadParams():
    """Get parameters from the parameter server."""
    global log_directory_name

    if rospy.has_param('/manipulation_client/experiment/logging/log_directory'):
        log_directory_name = rospy.get_param("/manipulation_client/experiment/logging/log_directory")
    else:
        rospy.logerr("Missing toggle logging service name (experiment/logging/log_directory)")
        return False

    return True


def getDir():
    """Get bag dir."""
    global log_directory_name
    rospack = rospkg.RosPack()

    return rospack.get_path("pr2_cartesian_clients") + "/data/" + log_directory_name


def openBag(name):
    """Open bag with given name in data/results."""
    bag = rosbag.Bag(getDir() + "/" + name + ".bag", 'r')
    return bag


if __name__ == '__main__':
    rospy.init_node('data_reader')
    gray = (0.85, 0.87, 0.89)
    num = 0

    mean_x_c_hat = np.array([])
    mean_error_x_c_hat = np.array([])
    mean_theta_c_hat = np.array([])
    mean_error_theta_c_hat = np.array([])
    mean_f_c_hat = np.array([])
    mean_error_f_c_hat = np.array([])

    if loadParams():
        for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
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
                bag_name = bag_prefix + str(num + 1)
                bag = openBag(bag_name)

                first = True
                i = 0
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

                    if num == 0 or len(mean_x_c_hat) <= i:
                        mean_x_c_hat = np.append(mean_x_c_hat, [msg.feedback.x_c_hat])
                        mean_error_x_c_hat = np.append(mean_error_x_c_hat, [msg.feedback.x_d - msg.feedback.x_c_hat])
                        mean_theta_c_hat = np.append(mean_theta_c_hat, [msg.feedback.theta_c_hat])
                        mean_error_theta_c_hat = np.append(mean_error_theta_c_hat, [msg.feedback.theta_d - msg.feedback.theta_c_hat])
                        mean_f_c_hat = np.append(mean_f_c_hat, [msg.feedback.f_c_hat])
                        mean_error_f_c_hat = np.append(mean_error_f_c_hat, [msg.feedback.f_d - msg.feedback.f_c_hat])
                    else:
                        mean_x_c_hat[i] = (mean_x_c_hat[i] + msg.feedback.x_c_hat)
                        mean_error_x_c_hat[i] = mean_error_x_c_hat[i] + msg.feedback.x_d - msg.feedback.x_c_hat
                        mean_theta_c_hat[i] = (mean_theta_c_hat[i] + msg.feedback.theta_c_hat)
                        mean_error_theta_c_hat[i] = mean_error_theta_c_hat[i] + msg.feedback.theta_d - msg.feedback.theta_c_hat
                        mean_f_c_hat[i] = (mean_f_c_hat[i] + msg.feedback.f_c_hat)
                        mean_error_f_c_hat[i] = mean_error_f_c_hat[i] + msg.feedback.f_d - msg.feedback.f_c_hat

                        i = i + 1

                # Print one set of results
                if len(t) > 0:
                    t = t - t[0]

                    plt.figure(1)
                    plt.subplot(321)
                    plt.plot(t, x_d - x_c_hat, color=gray)
                    plt.title('$x_c$ error')
                    plt.legend(['$x_d - \hat{x}_c$'])
                    plt.grid(True)

                    plt.subplot(322)
                    plt.plot(t, x_c_hat, color=gray)
                    plt.title('$x_c$ estimate')
                    plt.legend(['$\hat{x}_c$'])
                    plt.grid(True)

                    plt.subplot(323)
                    plt.plot(t, theta_d - theta_c_hat, color=gray)
                    plt.title('Theta error')
                    plt.legend(['$\\theta_d - \hat{\\theta}_c$'])
                    plt.grid(True)

                    plt.subplot(324)
                    plt.plot(t, theta_c_hat, color=gray)
                    plt.title('Angle estimate')
                    plt.legend(['$\hat{\\theta}_c$'])
                    plt.grid(True)

                    plt.subplot(325)
                    plt.plot(t, f_d - f_c_hat, color=gray)
                    plt.title('Force error')
                    plt.legend(['$f_d - \hat{f}_c$'])
                    plt.xlabel('Time [s]')
                    plt.grid(True)

                    plt.subplot(326)
                    plt.plot(t, f_c_hat, color=gray)
                    plt.title('Force estimate')
                    plt.legend(['$\hat{f}_c$'])
                    plt.xlabel('Time [s]')
                    plt.grid(True)

        mean_x_c_hat = mean_x_c_hat/(num + 1)
        mean_theta_c_hat = mean_theta_c_hat/(num + 1)
        mean_f_c_hat = mean_f_c_hat/(num + 1)
        mean_error_x_c_hat = mean_error_x_c_hat/(num + 1)
        mean_error_theta_c_hat = mean_error_theta_c_hat/(num + 1)
        mean_error_f_c_hat = mean_error_f_c_hat/(num + 1)

        plt.subplot(321)
        plt.plot(t, mean_error_x_c_hat[0:len(t)], 'k')
        plt.subplot(322)
        plt.plot(t, mean_x_c_hat[0:len(t)], 'k')
        plt.subplot(323)
        plt.plot(t, mean_error_theta_c_hat[0:len(t)], 'k')
        plt.subplot(324)
        plt.plot(t, mean_theta_c_hat[0:len(t)], 'k')
        plt.subplot(325)
        plt.plot(t, mean_error_f_c_hat[0:len(t)], 'k')
        plt.subplot(326)
        plt.plot(t, mean_f_c_hat[0:len(t)], 'k')
        plt.show()
