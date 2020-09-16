function [Q1, Q2] = solveIK(x,y)

    load('quad_params.mat');
    
    a_sq = (x - xc1).^2 + (y - yc1).^2;
    b_sq = (x - xc2).^2 + (y - yc2).^2;
    a = sqrt(a_sq);
    b = sqrt(b_sq);
    o1 = acos((-r2^2 + r1^2 + b_sq )./(2*r1*b));
    o2 = acos((-r2^2 + r1^2 + a_sq )./(2*r1*a));
    g1 = acos (-(a_sq - L^2 - b_sq )./(2*L*b));
    g2 = acos (-(b_sq - L^2 - a_sq )./(2*L*a));
    Q2 = -(g1+o1)*180/pi;
    Q1 = -(pi-(g2+o2))*180/pi;

end
