#!/usr/bin/env python

import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
import csv

def read_csv(filename):
    rows = []
    with open(filename, 'r') as csvfile:
    	csvreader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    	for row in csvreader:
    		rows.append(row)

    return np.array(rows, dtype=float)

def show_plots(desired, ground_truth):
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    maxx = np.max(np.concatenate((desired[:,0], ground_truth[:,0])))
    maxy = np.max(np.concatenate((desired[:,1], ground_truth[:,1])))
    minx = np.min(np.concatenate((desired[:,0], ground_truth[:,0])))
    miny = np.min(np.concatenate((desired[:,1], ground_truth[:,1])))
    ax.plot(desired[:,0],desired[:,1], 'r')
    ax.plot(ground_truth[:,0],ground_truth[:,1], 'b')
    plt.xlim(minx-1, maxx+1)
    plt.ylim(miny-0.1, maxy+0.1)
    plt.gca().set_aspect('equal', adjustable='datalim')
    plt.legend(('Desired', 'Actual'))
    plt.show()

    ts = 4
    tm = ground_truth.shape[0]%desired.shape[0]
    temp0 = desired[:,0]
    temp1 = desired[:,1]
    ttt = np.arange(0,1000, 0.002)
    ttt2 = np.arange(0,1000, 0.02)

    for i in range(ts-1):
        temp0 = np.concatenate((temp0,desired[:,0]))
        temp1 = np.concatenate((temp0,desired[:,1]))
    des_rep_0 = np.concatenate((temp0,desired[0:(tm),0]))
    des_rep_1 = np.concatenate((temp1,desired[0:(tm),1]))

    fig = plt.figure()
    ax  = fig.add_subplot(111)
    #maxx = np.max(np.concatenate((desired[:,0], ground_truth[:,0])))
    maxy = np.max(np.concatenate((desired[:,1], ground_truth[:,1])))
    #minx = np.min(np.concatenate((desired[:,0], ground_truth[:,0])))
    miny = np.min(np.concatenate((desired[:,1], ground_truth[:,1])))
    print(des_rep_0.shape)
    print(ttt.shape)
    ax.plot(ttt[0:des_rep_0.shape[0]], des_rep_0,'r')

    ax.plot(ttt2[0:ground_truth.shape[0]], ground_truth[:,0],'b')
    #plt.xlim(minx-1, maxx+1)
    #plt.ylim(miny-0.1, maxy+0.1)
    #plt.gca().set_aspect('equal', adjustable='datalim')
    plt.legend(('Desired', 'Actual'))
    plt.show()


    print("Done!")

if __name__ == '__main__':
    des_traj = read_csv("/home/arjun/Desktop/traj.csv").reshape(2000,2)
    ac_traj= read_csv("/home/arjun/Desktop/actraj.csv").reshape(800,2)

    show_plots(des_traj, ac_traj)
