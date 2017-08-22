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
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms

global log_directory_name, bag_prefix
log_directory_name = " "
bag_prefix = " "


def loadParams():
    """Get parameters from the parameter server."""
    global log_directory_name

    if rospy.has_param('/data_logger/log_directory'):
        log_directory_name = rospy.get_param("/data_logger/log_directory")
    else:
        rospy.logerr("Missing log directory (data_logger/log_directory)")
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


def saveFig(name):
    """Save the fig in the bag directory."""
    path = getDir() + "/" + name + ".svg"
    plt.savefig(path)


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
    # parser.add_argument("--all", action='store_true', help="True if meant to parse all")
    # parser.add_argument("--pivot", action='store_true', help="True if meant to plot pivot")
    # parser.add_argument("--new", action='store_true', help="True if results are from latest action version")

    args = parser.parse_args()

    # mean_x_c_hat = np.array([])
    mean_error_p_c = np.array([])
    mean_error_trans = np.array([])
    mean_error_rot = np.array([])

    matplotlib.rcParams['figure.figsize'] = (7, 5)
    matplotlib.rcParams['font.size'] = 14
    matplotlib.rcParams['lines.linewidth'] = 1
    matplotlib.rcParams['figure.subplot.wspace'] = 0.4
    matplotlib.rcParams['figure.subplot.hspace'] = 0.25

    if loadParams():

        if args.prefix is not None:
            bag_prefix = args.prefix

        if args.directory is not None:
            log_directory_name = args.directory

        print(bag_prefix)
        print(log_directory_name)

        for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
            t = np.array([])
            error_pc = np.array([])
            error_trans = np.array([])
            error_rot = np.array([])

            bag_name = bag_prefix + str(num + 1)
            bag = openBag(bag_name)

            first = True
            i = 0
            for topic, msg, time in bag.read_messages():
                t = np.append(t, [time.to_sec()])

                error_pc = np.append(error_pc, [msg.feedback.pc_distance_error])
                error_trans = np.append(error_trans, [msg.feedback.translational_angle_error])
                error_rot = np.append(error_rot, [msg.feedback.rotational_angle_error])

                if num == 0 or len(mean_error_p_c) <= i:
                    mean_error_p_c = np.append(mean_error_p_c, [msg.feedback.pc_distance_error])
                    mean_error_trans = np.append(mean_error_trans, [msg.feedback.translational_angle_error])
                    mean_error_rot = np.append(mean_error_rot, [msg.feedback.rotational_angle_error])

                else:
                    mean_error_p_c[i] = (mean_error_p_c[i] + msg.feedback.pc_distance_error)
                    mean_error_trans[i] = mean_error_trans[i] + msg.feedback.translational_angle_error
                    mean_error_rot[i] = (mean_error_rot[i] + msg.feedback.rotational_angle_error)

                    i = i + 1

            # Print one set of results
            if len(t) > 0:
                t = t - t[0]

                plt.figure(1)
                plt.subplot(311)
                plt.plot(t, error_pc, color=gray)
                plt.grid(True)

                plt.subplot(312)
                plt.plot(t, error_trans, color=gray)
                plt.grid(True)

                plt.subplot(313)
                plt.plot(t, error_rot, color=gray)
                plt.grid(True)

            else:
                print("error, no t")

        print(num)
        mean_error_p_c = mean_error_p_c/(num + 1)
        mean_error_trans = mean_error_trans/(num + 1)
        mean_error_rot = mean_error_rot/(num + 1)
        plt.figure(1)

        title_offset = 1.05
        t_final = 30

        plt.subplot(311)
        addLabelledPlot(t, mean_error_p_c[0:len(t)], "$\|p_c - \hat{p}_c\|$", 'k')
        plt.xlim(0.0, t_final)
        plt.ylim(-0.1, 0.2)
        plt.ylabel('[m]')
        plt.title('Contact point error norm error, $\|p_c - \hat{p}_c\|$', y=title_offset)

        plt.subplot(312)
        # addLabelledPlot(t, mean_x_c_hat[0:len(t)], '$\hat{x}_c$', 'k')
        plt.plot(t, mean_error_trans[0:len(t)], 'k')
        plt.xlim(0.0, t_final)
        plt.ylim(-0.8, 0.8)
        plt.title('Translational estimation error angle, $\hat{x}_c - x_c$', y=title_offset)
        plt.ylabel('[rad]')

        plt.subplot(313)
        plt.plot(t, mean_error_rot[0:len(t)], 'k')
        plt.title('Rotational estimation error angle, $f_d - \hat{f}_{c_y}$', y=title_offset)
        plt.xlim(0.0, t_final)
        plt.ylim(-0.8, 0.8)
        plt.ylabel('[rad]')
        plt.xlabel('Time [s]')
        # plt.subplot(313)
        # addLabelledPlot(t, f_c_hat[0:len(t)], '$\hat{f}_c$', 'k')
        plt.tight_layout()
        saveFig(bag_prefix)
        plt.show()
