import numpy as np
import matplotlib.pyplot as plt
import time

    # ..... Geometrical .....
r1              = 10.0               # length of Cranks
r2              = 20.0               # length of Coupler Links
L               = 5.0                # Distance between the pivot points
base_l          = 25.0               # Base length
base_w          = 25.0               # Base width
d               = 0.0                # Offset distance between side edge and foot center
xc1 =  L/2.0; yc1 = 0.0              # Joint-1 coordinates
xc2 = -L/2.0; yc2 = 0.0              # Joint-2 coordinates

    # ..... Trajectory .....
stroke_h        = 18.0               # Robot Height
ht              = 6.0                # Step height

    # ..... Temporal .....
def_period      = 1.5                # default period
dc              = 0.8                # Duty factor
dt              = 0.03               # Extension time
phi1            = +0.5               # Phase diff. b/w Contralateral legs (Leg 1 and Leg 4, Leg 2 and Leg 3)
phi2            = +0.75              # Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3, Leg 2 and Leg 4)
step_size       = 0.002
leg_phase       = np.array([0, 0.5, 0.25, 0.75], dtype=float)


def getParams(lin_vel, ang_vel):

    #params = [stroke, ht, period]
    l_params = np.ndarray([3])
    r_params = np.ndarray([3])

    vl = lin_vel - base_w*ang_vel/2
    vr = lin_vel + base_w*ang_vel/2

    l_params[0] = def_period*vl/dc
    r_params[0] = def_period*vr/dc

    l_params[1] = 2.5*abs(l_params[0])/8 + 1
    r_params[1] = 2.5*abs(r_params[0])/8 + 1

    l_params[2] = def_period
    r_params[2] = def_period

    return [l_params, r_params]

    return paramTraj

def getTrajectory(paramTraj):

    stroke, ht, period = paramTraj

    C_x = np.array([              - (stroke/2) - (dt*stroke)/(dc*period),
                                                    - stroke/(dc*period),
                      (3*stroke)/(dc*pow((2*dt - period + dc*period),2)),
                     (2*stroke)/(dc*pow((2*dt - period + dc*period),3))],
                                                             dtype=float)

    C_y = np.array([                               -stroke_h,
                                                           0,
                            (16*ht)/pow((period*(dc - 1)),2),
                            (32*ht)/pow((period*(dc - 1)),3),
                           (16*ht)/pow((period*(dc - 1)),4)],
                                                 dtype=float)


    n = int(period/step_size)
    Time = np.arange(0,period,step_size)
    X = np.zeros(n); Y = np.zeros(n)

    #.......... X vs Time ..........#
    Phase_x = np.array([((0.5*dc*period+dt)/step_size), ((period-(dt+0.5*dc*period))/step_size)], dtype=int)
    X[0:Phase_x[0]] = -stroke*Time[0:Phase_x[0]]/(dc*period)
    t = Time[Phase_x[0]:Phase_x[1]] - (0.5*dc*period+dt)
    X[Phase_x[0]:Phase_x[1]] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3))).T),C_x)
    X[Phase_x[1]:] = stroke*(period - Time[Phase_x[1]:])/(dc*period)

    #.......... Y vs Time ..........#
    Phase_y = np.array([(0.5*dc*period)/step_size, ((1 - 0.5*dc)*period)/step_size], dtype=int)
    Y[0:Phase_y[0]] = -1*stroke_h*np.ones(Phase_y[0])
    t = Time[Phase_y[0]:Phase_y[1]] - (0.5*dc*period)
    Y[Phase_y[0]:Phase_y[1]] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3), np.power(t,4))).T),C_y)
    Y[Phase_y[1]:] = -1*stroke_h*np.ones(n-Phase_y[1])
    #return np.column_stack((X, Y, Time))
    return [X, Y, Time]

def getTrajectory2(paramTraj):

    stroke, ht, period = paramTraj

    C_x = np.array([              - (stroke/2) - (dt*stroke)/(dc*period),
                                                    - stroke/(dc*period),
                      (3*stroke)/(dc*pow((2*dt - period + dc*period),2)),
                     (2*stroke)/(dc*pow((2*dt - period + dc*period),3))],
                                                             dtype=float)

    C_y = np.array([                               -stroke_h,
                                                           0,
                            (16*ht)/pow((period*(dc - 1)),2),
                            (32*ht)/pow((period*(dc - 1)),3),
                           (16*ht)/pow((period*(dc - 1)),4)],
                                                 dtype=float)


    n = int(period/step_size)
    Time = np.arange(0,period,step_size)
    X = np.zeros(n); Y = np.zeros(n)

    #.......... X vs Time ..........#
    Phase_x = np.array([((dc*period+dt)/step_size), ((period-dt)/step_size)], dtype=int)
    X[0:Phase_x[0]] = stroke/2 - stroke*Time[0:Phase_x[0]]/(dc*period)
    t = Time[Phase_x[0]:Phase_x[1]] - (dc*period+dt)
    X[Phase_x[0]:Phase_x[1]] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3))).T),C_x)
    X[Phase_x[1]:] = stroke/2 + stroke*(period - Time[Phase_x[1]:])/(dc*period)

    #.......... Y vs Time ..........#
    Phase_y = np.array([(dc*period)/step_size, ((dc*period)/step_size+1)], dtype=int)
    Y[0:Phase_y[0]] = -1*stroke_h*np.ones(Phase_y[0])
    t = Time[Phase_y[0]:] - (dc*period)
    Y[Phase_y[0]:] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3), np.power(t,4))).T),C_y)

    #return np.column_stack((X, Y, Time))
    return [X, Y, Time]

def subplotTraj(x,y,idx,name, labels=None):
    plt.subplot(idx)
    plt.plot(x, y);
    plt.title(name);
    if not labels == None:
        plt.xlabel(labels[0])
        plt.ylabel(labels[1])

def plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r):

    subplotTraj(Time_l, X_l, 131, 'X vs Time', ["time (s)", "X (cm)"])
    subplotTraj(Time_l, Y_l, 132, 'Y vs Time', ["time (s)", "Y (cm)"])
    subplotTraj(X_l, Y_l, 133, 'Trajectory', ["Y (cm)", "X (cm)"])

    '''subplotTraj(Time_r, X_r, 234, 'Right X vs Time')
    subplotTraj(Time_r, Y_r, 235, 'Right Y vs Time')
    subplotTraj(X_r, Y_r, 236, 'Right Trajectory')'''

    #plt.plot(X_r, Y_r)

    plt.show()


def solveIK(x,y):

    a_sq = np.power((x - xc1),2) + np.power((y - yc1),2);
    b_sq = np.power((x - xc2),2) + np.power((y - yc2),2);
    a = np.sqrt(a_sq)
    b = np.sqrt(b_sq)

    o1 = np.arccos((-(r2*r2) + (r1*r1) + b_sq)/(2*r1*b));
    o2 = np.arccos((-(r2*r2) + (r1*r1) + a_sq)/(2*r1*a));
    g1 = np.arccos(-(a_sq - (L*L) - b_sq)/(2*L*b));
    g2 = np.arccos(-(b_sq - (L*L) - a_sq)/(2*L*a));
    Q2 = -(g1+o1)*180/np.pi;
    Q1 = -(np.pi-(g2+o2))*180/np.pi;

    return [Q1, Q2]

def saveData(pos, var):
    np.savetxt(pos, var, delimiter=",")


if __name__ == "__main__":

    start_time = time.time()

    lin_vel = 20
    ang_vel = 0.00

    l_params, r_params = getParams(lin_vel, ang_vel)

    #print(l_params)
    #print(r_params)

    [X_l, Y_l, Time_l] = getTrajectory(l_params)
    [X_r, Y_r, Time_r] = getTrajectory(r_params)

    plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r)

    [Q1_l, Q2_l] = solveIK(X_l,Y_l)
    [Q1_r, Q2_r] = solveIK(X_r,Y_r)


    print("--- %s seconds ---" % (time.time() - start_time))
