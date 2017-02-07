#!/usr/bin/env python
"""Provide a node that will read data from logged bags"""
from __future__ import (absolute_import, division, print_function,
                        unicode_literals)
import rospy
import rosbag
import rospkg
import argparse
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms

global log_directory_name, bag_prefix
log_directory_name = " "
bag_prefix = " "


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


def autoscaleBasedOn(ax, lines):
    """Autoscale the given axis according to the given lines.

    From http://stackoverflow.com/questions/7386872/make-matplotlib-autoscaling-ignore-some-of-the-plots
    """
    ax.dataLim = mtransforms.Bbox([[0, 0], [0.0001, 0.0001]])
    for line in lines:
        xy = np.vstack(line.get_data()).T
        ax.dataLim.update_from_data_xy(xy, ignore=False)
    ax.autoscale_view()


def addLabelledPlot(x, y, label, color='k'):
    """Add a plot with the given label."""
    handle, = plt.plot(x, y, color)
    axes = plt.gca()
    autoscaleBasedOn(axes, [handle])
    # plt.legend([handle], [label])

if __name__ == '__main__':
    rospy.init_node('data_reader')
    gray = (0.85, 0.87, 0.89)
    num = 0

    parser = argparse.ArgumentParser(description="""Data reader arguments""")
    parser.add_argument("--prefix", type=str, help="The bag prefix")
    parser.add_argument("--directory", type=str, help="The bag directory")
    parser.add_argument("--all", action='store_true', help="True if meant to parse all")

    args = parser.parse_args()
    plot_all = args.all

    mean_x_c_hat = np.array([])
    mean_error_x_c_hat = np.array([])
    mean_theta_c_hat = np.array([])
    mean_error_theta_c_hat = np.array([])
    mean_f_c_hat = np.array([])
    mean_error_f_c_hat = np.array([])

    if loadParams():

        if args.prefix is not None:
            bag_prefix = args.prefix

        if args.directory is not None:
            log_directory_name = args.directory

        print(bag_prefix)
        print(log_directory_name)

        for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
            t = np.array([])
            x_c_hat = np.array([])
            x_c = np.array([])
            x_e = np.array([])
            y_e = np.array([])
            x_d = np.array([])
            var_x = np.array([])
            theta_c_hat = np.array([])
            theta_c = np.array([])
            theta_d = np.array([])
            var_theta = np.array([])
            f_c = np.array([])
            f_c_hat = np.array([])
            var_f = np.array([])
            f_d = np.array([])

            bag_name = bag_prefix + str(num + 1)
            bag = openBag(bag_name)

            first = True
            i = 0
            for topic, msg, time in bag.read_messages():
                t = np.append(t, [time.to_sec()])

                if plot_all:
                    x_c_hat = np.append(x_c_hat, [msg.feedback.x_c_hat])
                    x_c = np.append(x_c, [msg.feedback.x_c_2])
                    x_e = np.append(x_e, [msg.feedback.x_e])
                    y_e = np.append(y_e, [msg.feedback.y_e])
                    x_d = np.append(x_d, [msg.feedback.x_d])
                    var_x = np.append(var_x, [msg.feedback.var_x])
                    theta_c_hat = np.append(theta_c_hat, [msg.feedback.theta_c_hat])
                    theta_c = np.append(theta_c, [msg.feedback.theta_c_2])
                    theta_d = np.append(theta_d, [msg.feedback.theta_d])
                    var_theta = np.append(var_theta, [msg.feedback.var_theta])

                f_c = np.append(f_c, [msg.feedback.f_c])
                f_c_hat = np.append(f_c_hat, [msg.feedback.f_c_hat])
                f_d = np.append(f_d, [msg.feedback.f_d])
                var_f = np.append(var_f, [msg.feedback.var_f])

                if num == 0 or len(mean_x_c_hat) <= i:
                    if plot_all:
                        mean_x_c_hat = np.append(mean_x_c_hat, [msg.feedback.x_c_hat - msg.feedback.x_c_2])
                        # mean_error_x_c_hat = np.append(mean_error_x_c_hat, [msg.feedback.x_d - msg.feedback.x_c_hat])
                        mean_error_x_c_hat = np.append(mean_error_x_c_hat, [msg.feedback.x_d - msg.feedback.x_c_2])
                        mean_theta_c_hat = np.append(mean_theta_c_hat, [msg.feedback.theta_c_hat - msg.feedback.theta_c_2])
                        # mean_error_theta_c_hat = np.append(mean_error_theta_c_hat, [msg.feedback.theta_d - msg.feedback.theta_c_hat])
                        mean_error_theta_c_hat = np.append(mean_error_theta_c_hat, [msg.feedback.theta_d - msg.feedback.theta_c_2])
                    mean_f_c_hat = np.append(mean_f_c_hat, [msg.feedback.f_c_hat])
                    mean_error_f_c_hat = np.append(mean_error_f_c_hat, [msg.feedback.f_d - msg.feedback.f_c_hat])
                else:
                    if plot_all:
                        mean_x_c_hat[i] = (mean_x_c_hat[i] + msg.feedback.x_c_hat - msg.feedback.x_c_2)
                        # mean_error_x_c_hat[i] = mean_error_x_c_hat[i] + msg.feedback.x_d - msg.feedback.x_c_hat
                        mean_error_x_c_hat[i] = mean_error_x_c_hat[i] + msg.feedback.x_d - msg.feedback.x_c_2
                        mean_theta_c_hat[i] = (mean_theta_c_hat[i] + msg.feedback.theta_c_hat - msg.feedback.theta_c_2)
                        # mean_error_theta_c_hat[i] = mean_error_theta_c_hat[i] + msg.feedback.theta_d - msg.feedback.theta_c_hat
                        mean_error_theta_c_hat[i] = mean_error_theta_c_hat[i] + msg.feedback.theta_d - msg.feedback.theta_c_2

                    mean_f_c_hat[i] = (mean_f_c_hat[i] + msg.feedback.f_c_hat)
                    mean_error_f_c_hat[i] = mean_error_f_c_hat[i] + msg.feedback.f_d - msg.feedback.f_c_hat

                    i = i + 1

            # Print one set of results
            if len(t) > 0:
                t = t - t[0]

                if plot_all:
                    plt.figure(1)
                    plt.subplot(321)
                    plt.plot(t, x_d - x_c, color=gray)
                    plt.ylabel('[m]')
                    plt.title('Translational error, $x_d - x_c$')
                    plt.grid(True)

                    plt.subplot(222)
                    plt.plot(t, x_c_hat - x_c, color=gray)
                    plt.title('Translational estimation error, $\hat{x}_c - x_c$')
                    plt.ylabel('[m]')
                    plt.grid(True)

                    plt.subplot(323)
                    plt.plot(t, theta_d - theta_c, color=gray)
                    plt.ylabel('[rad]')
                    plt.title('Orientation error, $\\theta_d - \\theta_c$')
                    plt.grid(True)

                    plt.subplot(224)
                    plt.plot(t, theta_c_hat - theta_c, color=gray)
                    plt.ylabel('[rad]')
                    plt.title('Angle estimation error, $\hat{\\theta}_c - \\theta_c$')
                    plt.xlabel('Time [s]')
                    plt.grid(True)

                    plt.subplot(325)
                    plt.plot(t, f_d - f_c_hat, color=gray)
                    plt.title('Force error, $f_d - \hat{f}_c$')
                    plt.ylabel('[N]')
                    plt.xlabel('Time [s]')
                    plt.grid(True)

                # plt.subplot(313)
                # plt.plot(t, f_c, color=gray)
                # plt.title('Force estimate')
                # plt.xlabel('Time [s]')
                # plt.grid(True)

                # plt.figure(2)
                # plt.subplot(311)
                # plt.plot(t, x_c, 'k')
                # plt.plot(t, x_e, 'b')
                # plt.title('$x_c$')
                # plt.grid(True)
                # plt.subplot(312)
                # plt.plot(t, y_e, 'b')
                # plt.title('$y_e$')
                # plt.grid(True)
                # plt.subplot(313)
                # plt.plot(t, theta_c, 'k')
                # plt.plot(t, theta_c_hat, 'b')
                # plt.title('$\\theta_c$')
                # plt.xlabel('Time [s]')
                # plt.grid(True)
            else:
                print("error, no t")

        print(num)
        mean_x_c_hat = mean_x_c_hat/(num + 1)
        mean_theta_c_hat = mean_theta_c_hat/(num + 1)
        mean_f_c_hat = mean_f_c_hat/(num + 1)
        mean_error_x_c_hat = mean_error_x_c_hat/(num + 1)
        mean_error_theta_c_hat = mean_error_theta_c_hat/(num + 1)
        mean_error_f_c_hat = mean_error_f_c_hat/(num + 1)

        plt.figure(1)

        if plot_all:
            plt.subplot(321)
            addLabelledPlot(t, mean_error_x_c_hat[0:len(t)], "$x_d - \hat{x}_c$", 'k')
            plt.subplot(222)
            addLabelledPlot(t, mean_x_c_hat[0:len(t)], '$\hat{x}_c$', 'k')
            plt.subplot(323)
            addLabelledPlot(t, mean_error_theta_c_hat[0:len(t)], '$\\theta_d - \hat{\\theta}_c$', 'k')
            plt.subplot(224)
            addLabelledPlot(t, mean_theta_c_hat[0:len(t)], '$\hat{\\theta}_c$', 'k')
            plt.subplot(325)
            addLabelledPlot(t, mean_error_f_c_hat[0:len(t)], '$f_d - \hat{f}_c$', 'k')
        else:
            plt.plot(t, f_c, color=gray)
            plt.plot(t, f_c_hat, 'k')
            plt.title('Force estimate')
            plt.xlabel('Time [s]')
            plt.grid(True)
        #
        # plt.subplot(313)
        # addLabelledPlot(t, f_c_hat[0:len(t)], '$\hat{f}_c$', 'k')
        # plt.tight_layout()
        plt.show()
