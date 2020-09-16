format long

tic

lin_vel = 20;
ang_vel = 0.0;
    
params = getParams(lin_vel, ang_vel);
l_params = params(:,1);
r_params = params(:,2);

[X_l, Y_l, Time_l] = getTrajectory(l_params);
[X_r, Y_r, Time_r] = getTrajectory(r_params);

% csvwrite('/home/arjun/Desktop/minitaur/trajectory_codes/python/comparefiles/matlabX_l.txt',X_l)
% csvwrite('/home/arjun/Desktop/minitaur/trajectory_codes/python/comparefiles/matlabY_l.txt',Y_l)
% csvwrite('/home/arjun/Desktop/minitaur/trajectory_codes/python/comparefiles/matlabTime_l.txt',Time_l)


plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r);

[Q1_l, Q2_l] = solveIK(X_l,Y_l);
[Q1_r, Q2_r] = solveIK(X_r,Y_r);

% animateMotionPlan(Q1_l, Q2_l, X_l, Y_l);
% animateMotionPlan(Q1_r, Q2_r, X_r, Y_r);

%checkStability(X_l, Y_l, X_r, Y_r, true)

toc

