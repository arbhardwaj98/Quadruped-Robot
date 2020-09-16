function [] = plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r)
    
    subplot(2,3,1)
    plot(Time_l, X_l);
    title('Left X vs Time');
    xlabel('t'); ylabel('x_left');

    subplot(2,3,2)
    plot(Time_l, Y_l);
    title('Left Y vs Time');
    xlabel('t'); ylabel('y_left');
    
    subplot(2,3,3)
    plot(X_l, Y_l);
    title('Left Trajectory');
    xlabel('x_left'); ylabel('y_left');

    subplot(2,3,4)
    plot(Time_r, X_r);
    title('Right X vs Time');
    xlabel('t'); ylabel('x_right');

    subplot(2,3,5)
    plot(Time_r, Y_r);
    title('Right Y vs Time');
    xlabel('t'); ylabel('y_right');
    
    subplot(2,3,6)
    plot(X_r, Y_r);
    title('Right Trajectory');
    xlabel('x_right'); ylabel('y_right');
    

end