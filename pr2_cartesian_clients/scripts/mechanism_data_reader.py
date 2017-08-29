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
    handle, = plt.plot(x, y, color, label=label)
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
    parser.add_argument("--oneonly", action='store_true', help="True if meant to display only one")
    parser.add_argument("--alpha", action='store_true', help="Plot alpha histogram")
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
    t_final = 19    
    title_offset = 1.05
    label_h = 1
    label_v = 1
    loc = 'upper right'

    if loadParams():

        if args.prefix is not None:
            bag_prefix = args.prefix

        if args.directory is not None:
            log_directory_name = args.directory

        print(bag_prefix)
        print(log_directory_name)
        
        if args.oneonly:
            t = np.array([])
            error_pc = np.array([])
            error_trans = np.array([])
            error_rot = np.array([])
            vs = np.array([])
            vforce = np.array([])
            wr = np.array([])
            bag_name = bag_prefix
            bag = openBag(bag_name)
            
            for topic, msg, time in bag.read_messages():
                t = np.append(t, [time.to_sec()])

                error_pc = np.append(error_pc, [msg.feedback.pc_distance_error])
                error_trans = np.append(error_trans, [msg.feedback.translational_angle_error])
                error_rot = np.append(error_rot, [msg.feedback.rotational_angle_error])
                # print(msg.feedback.relative_twist.twist.linear.x**2 + msg.feedback.relative_twist.twist.linear.y**2 + msg.feedback.relative_twist.twist.linear.z**2)
                vs = np.append(vs, [msg.feedback.vs])
                vforce = np.append(vforce, [msg.feedback.vforce])
                wr = np.append(wr, [msg.feedback.wr])

            # Print one set of results
            if len(t) > 0:
                t = t - t[0]
                
                matplotlib.rcParams['figure.figsize'] = (14, 9)
                plt.figure(1)
                plt.subplot(411)
                plt.plot(t, vs, 'k', label="$v_s$")
                plt.plot(t, vforce, 'r', label= "$\mathbf{v}_f^\\top \mathbf{k}$ ")
                plt.grid(True)
                plt.xlim(0.0, t_final)
                plt.ylim(-0.05, 0.05)
                plt.ylabel('[m/s]')
                # plt.title('Commanded velocities', y=title_offset)
                plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
                # plt.legend(loc=0, fontsize='medium')
                
                plt.subplot(412)
                plt.plot(t, wr, 'k', label='$\\omega_r$')
                plt.grid(True)
                plt.xlim(0.0, t_final)
                plt.ylim(-0.1, 0.1)
                plt.ylabel('[rad/s]')
                plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
                # plt.title('Commanded relative angular velocity, $\\omega_r$', y=title_offset)
                
                plt.subplot(413)
                plt.plot(t, error_pc, 'k', label='$\|\mathbf{p}_c - \hat{\mathbf{p}}_c\|$')
                plt.grid(True)
                plt.xlim(0.0, t_final)
                plt.ylim(0.0, 0.1)
                plt.ylabel('[m]')
                plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
                # plt.title('Contact point error norm error, $\|\mathbf{p}_c - \hat{\mathbf{p}}_c\|$', y=title_offset)

                plt.subplot(414)
                plt.plot(t, error_trans, 'k', label='$\\theta_t$')
                plt.grid(True)
                plt.xlim(0.0, t_final)
                plt.ylim(0.0, 0.7)
                plt.ylabel('[rad]')
                plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
                # plt.title('Translational DOF angle error $\\theta_t$', y=title_offset)

                # plt.subplot(313)
                # plt.plot(t, error_rot, color=gray)
                # plt.grid(True)

            else:
                print("error, no t")
        
        elif args.alpha:
            
            #open first bag and check alphas
            alphas = np.array([])
            
            for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
                bag_name = bag_prefix + str(num + 1)
                bag = openBag(bag_name)
                end_time = bag.get_end_time()
                
                for topic, msg, time in bag.read_messages(start_time = rospy.Time(end_time)):
                    alphas = np.append(alphas, [msg.feedback.alpha])
            
            print(alphas)
            alpha_values = np.unique(alphas)
            num_bars = len(alpha_values)
            error_pc = np.zeros([num_bars])
            dev_pc = np.zeros([num_bars])
            error_trans = np.zeros([num_bars])
            dev_trans = np.zeros([num_bars])
            total = np.zeros([num_bars])
            
            for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
                bag_name = bag_prefix + str(num + 1)
                bag = openBag(bag_name)
                end_time = bag.get_end_time()
                print(num)
                print(end_time)
                
                for topic, msg, time in bag.read_messages(start_time = rospy.Time(end_time)):
                    alpha = msg.feedback.alpha
                    
                    index = np.where(alpha_values == alpha)[0][0]
                    
                    error_pc[index] = error_pc[index] + msg.feedback.pc_distance_error
                    error_trans[index] = error_trans[index] + msg.feedback.translational_angle_error
                    total[index] = total[index] + 1 
                    break
                    # print("hehe")
                    
            for i in range(0, len(error_pc)):
                error_pc[i] = error_pc[i]/(total[i])
                error_trans[i] = error_trans[i]/(total[i])
                
            for num in range(len(glob.glob(getDir() + "/" + bag_prefix + "*.bag"))):
                bag_name = bag_prefix + str(num + 1)
                bag = openBag(bag_name)
                end_time = bag.get_end_time()
                print(num)
                print(end_time)
                
                for topic, msg, time in bag.read_messages(start_time = rospy.Time(end_time)):
                    alpha = msg.feedback.alpha
                    
                    index = np.where(alpha_values == alpha)[0][0]
                    
                    dev_pc[index] = dev_pc[index] + (msg.feedback.pc_distance_error - error_pc[index])**2
                    dev_trans[index] = dev_trans[index] + (msg.feedback.translational_angle_error - error_trans[index])**2
                    break
                    
            
            for i in range(0, len(dev_pc)):
                print("total")
                print(total[i])
                dev_pc[i] = np.sqrt(dev_pc[i]/(total[i]))
                dev_trans[i] = np.sqrt(dev_trans[i]/(total[i]))
                
            plt.figure(1)
            plt.subplot(211)
            plt.bar(alpha_values - 0.025, error_pc, width=0.05, yerr = dev_pc, ecolor='k')
            plt.xlim(-0.025, 1.025)
            plt.ylabel('[m]')
            plt.grid(True)
            
            plt.subplot(212)
            plt.bar(alpha_values - 0.025, error_trans, width=0.05, yerr = dev_trans, ecolor='k')
            plt.ylabel('[rad]')
            plt.xlim(-0.025, 1.025)
            plt.xlabel('$\\alpha$')
            plt.grid(True)
            
        else:

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
                    plt.subplot(211)
                    plt.plot(t, error_pc, color=gray)
                    plt.grid(True)

                    plt.subplot(212)
                    plt.plot(t, error_trans, color=gray)
                    plt.grid(True)

                    # plt.subplot(313)
                    # plt.plot(t, error_rot, color=gray)
                    # plt.grid(True)

                else:
                    print("error, no t")

            print(num)
            mean_error_p_c = mean_error_p_c/(num + 1)
            mean_error_trans = mean_error_trans/(num + 1)
            mean_error_rot = mean_error_rot/(num + 1)
            plt.figure(1)

            plt.subplot(211)
            addLabelledPlot(t, mean_error_p_c[0:len(t)], "$\|\mathbf{p}_c - \hat{\mathbf{p}}_c\|$", 'k')
            plt.xlim(0.0, t_final)
            plt.ylim(0.0, 0.15)
            plt.ylabel('[m]')
            # plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
            
            # plt.title('Contact point error norm error, $\|\mathbf{p}_c - \hat{\mathbf{p}}_c\|$', y=title_offset)

            plt.subplot(212)
            addLabelledPlot(t, mean_error_trans[0:len(t)], '$\\theta_{t}$', 'k')
            plt.xlim(0.0, t_final)
            plt.ylim(0.0, 0.7)
            # plt.legend(bbox_to_anchor=(label_h, label_v), loc=loc, ncol=2, fontsize='medium', fancybox=True, shadow=True)
            # plt.title('Translational estimation error angle, $\\theta_{t}$', y=title_offset)
            plt.ylabel('[rad]')

            # plt.subplot(313)
            # plt.plot(t, mean_error_rot[0:len(t)], 'k')
            # plt.title('Rotational estimation error angle, $f_d - \hat{f}_{c_y}$', y=title_offset)
            # plt.xlim(0.0, t_final)
            # plt.ylim(-0.8, 0.8)
            # plt.ylabel('[rad]')
            # plt.xlabel('Time [s]')
            # plt.subplot(313)
            # addLabelledPlot(t, f_c_hat[0:len(t)], '$\hat{f}_c$', 'k')
    plt.tight_layout()
    saveFig(bag_prefix)
    plt.show()
