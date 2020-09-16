function [X, Y, Time] = getTrajectory(paramTraj)

    load('quad_params.mat');
    stroke = paramTraj(1);
    ht = paramTraj(2);
    period = paramTraj(3);
    
    C_x = [          - (stroke/2) - (dt*stroke)/(dc*period);
                                       - stroke/(dc*period);
              (3*stroke)/(dc*(2*dt - period + dc*period)^2);
             (2*stroke)/(dc*(2*dt - period + dc*period)^3)];

    C_y = [                       -stroke_h;
                                          0;
              (16*ht)/(period^2*(dc - 1)^2);
              (32*ht)/(period^3*(dc - 1)^3);
             (16*ht)/(period^4*(dc - 1)^4)];

    n = period/step_size;

    Time = (0:step_size:(period-step_size))';
    X = zeros(n, 1); Y = zeros(n, 1);

    %.......... X vs Time ..........%
    Phase_x = round([((dc*period+dt)/step_size); ((period-dt)/step_size)]);
    X(1:Phase_x(1)) = stroke/2 - stroke*Time(1:Phase_x(1))/(dc*period);
    t = Time(Phase_x(1)+1:Phase_x(2)) - (dc*period+dt);
    X(Phase_x(1)+1:Phase_x(2)) = [ones(size(t,1),1), t, t.^2, t.^3]*C_x;
    X(Phase_x(2)+1:end) = stroke/2 + stroke*(period - Time(Phase_x(2)+1:end))/(dc*period);

    %.......... Y vs Time ..........%
    Phase_y = round([(dc*period)/step_size, (dc*period)/step_size+1]);
    Y(1:Phase_y(1)) = -1*stroke_h*ones(Phase_y(1),1);
    t = Time((Phase_y(1)+1):end) - (dc*period);
    Y((Phase_y(1)+1):end) = [ones(size(t,1),1), t, t.^2, t.^3, t.^4]*C_y;
    
end