#!/usr/bin/env python

import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
import csv

test_name = "straight_line2"

dir = os.path.join("/home/arjun/quadruped_ws/src/quadruped_driver/results_csvs", test_name)
if not os.path.isdir(dir):
    print ("Directory does Not Exist")
    exit()

def read_csv(filename):
    rows = []
    with open(filename, 'r') as csvfile:
    	csvreader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    	for row in csvreader:
    		rows.append(row)

    return np.array(rows, dtype=float)

def get_desired(sim_time, twist, ground_truth):
    '''quat = Quaternion()
    quat.x = ground_truth[0,4]; quat.x = ground_truth[0,5]; quat.x = ground_truth[0,6]; quat.x = ground_truth[0,7]'''
    quat = ground_truth[0,4:]
    rpy = tf.transformations.euler_from_quaternion(quat)
    phi = rpy[2]
    t = np.arange(0, sim_time[0,1]-sim_time[0,0], 0.01).reshape(-1,1)
    if twist[0,1]!=0:
        R = twist[0,0]/(0.6*twist[0,1]) ##################################
        C = ground_truth[0,1:3] - np.array([R*np.sin(phi), -R*np.cos(phi)]).reshape(1,2)
        C2 = C
        a = np.hstack((np.cos(twist[0,1]*t[:,0] + phi-np.pi/2).reshape(-1,1), np.sin(twist[0,1]*t[:,0] + phi-np.pi/2).reshape(-1,1)))
        traj = C2 + R*a
        traj2 = C2 + 0.05*a
        o = phi + 0.36*t*twist[0,1]
    else:
        traj = ground_truth[0,1:3] + t[:,0].reshape(-1,1)*twist[0,0]*np.array([np.cos(phi), np.sin(phi)]).reshape(1,2)
        o = phi*np.ones(t.shape)

    traj = np.concatenate((traj.reshape(-1,2), o), axis=1)
    return traj

def show_plots(sim_time, desired, ground_truth):
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    maxx = np.max(np.concatenate((desired[:,0], ground_truth[:,1])))
    maxy = np.max(np.concatenate((desired[:,1], ground_truth[:,2])))
    minx = np.min(np.concatenate((desired[:,0], ground_truth[:,1])))
    miny = np.min(np.concatenate((desired[:,1], ground_truth[:,2])))
    ax.plot(desired[:,0],desired[:,1], 'r')
    ax.plot(ground_truth[:,1],ground_truth[:,2], 'b')
    plt.xlim(minx-0.1, maxx+0.1)
    plt.ylim(miny-0.1, maxy+0.1)
    plt.gca().set_aspect('equal', adjustable='datalim')
    plt.legend(('Desired', 'Actual'))
    plt.title("Trajectory")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.show()

    rpy = np.zeros((ground_truth.shape[0],3))
    for i in range(ground_truth.shape[0]):
        rpy[i,:] = tf.transformations.euler_from_quaternion(ground_truth[i,4:])
    phi = (180*rpy[:,2]/np.pi)%360
    desired[:,2] = (180*desired[:,2]/np.pi)%360
    t_des = np.arange(0, sim_time[0,1]-sim_time[0,0], 0.01).reshape(-1,1)
    t = ground_truth[:,0].reshape(-1,1) - ground_truth[0,0]
    maxy = max(np.max(phi[:]), np.max(desired[:,2]))
    miny = min(np.min(phi[:]), np.min(desired[:,2]))
    plt.plot(t_des, desired[:,2] ,'r', t, phi, 'b')
    plt.xlim(0, t[-1,0])
    plt.ylim(miny-5, maxy+5)
    plt.legend(('Desired', 'Actual'))
    plt.title("Orientation")
    plt.ylabel("${\Theta}$ (deg)")
    plt.xlabel("time (s)")
    plt.show()

    fig = plt.figure()
    ax  = fig.add_subplot(111)
    mn = np.mean(ground_truth[:,3])
    t = ground_truth[:,0].reshape(-1,1) - ground_truth[0,0]
    des = mn*np.zeros(ground_truth[:,0].shape)
    maxy = np.max(ground_truth[:,3]-mn)
    miny = np.min(ground_truth[:,3]-mn)
    ax.plot(t, des ,'r',linewidth=3.0)
    ax.plot(t, ground_truth[:,3]-mn, 'b', t, des-0.005 ,'--y', t, des+0.005 ,'--y')
    plt.xlim(0, t[-1,0])
    plt.ylim(miny-0.01, maxy+0.01)
    ax.legend(('Desired', 'Actual'))
    plt.title("COM Stability")
    plt.ylabel("z (m)")
    plt.xlabel("time (s)")
    plt.show()

    print("Done!")

if __name__ == '__main__':
    twist = read_csv(os.path.join(dir, "twist.csv")).reshape(1,2)
    sim_time = read_csv(os.path.join(dir, "sim_time.csv")).reshape(1,2)
    ground_truth = read_csv(os.path.join(dir, "ground_truth.csv"))
    desired = get_desired(sim_time, twist, ground_truth)

    show_plots(sim_time, desired, ground_truth)
