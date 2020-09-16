function [] = animateMotionPlan(Q1, Q2, x, y)
        
    load('quad_params.mat');

    xa = xc1 + r1 * cosd(Q1);
    ya = yc1 + r1 * sind(Q1);
    xb = xc2 + r1 * cosd(Q2);
    yb = yc2 + r1 * sind(Q2);
    
    for i = 1:length(x)
        
        figure(1)
        plot([xc2,xb(i),x(i),xa(i),xc1],[yc2,yb(i),y(i),ya(i),yc1]);
        xlim([-15,15]);
        ylim([-23, 7]);
        pause(step_size);
        
    end

end
