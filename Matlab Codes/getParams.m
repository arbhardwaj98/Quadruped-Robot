function [paramTraj] = getParams(lin_vel, ang_vel)

    load('quad_params.mat');
    
    %params = [stroke, ht, period] 
    l_params = zeros(3,1);
    r_params = zeros(3,1);
    
    vl = lin_vel - base_w*ang_vel/2;
    vr = lin_vel + base_w*ang_vel/2;
    
    l_params(1) = def_period*vl/dc;
    r_params(1) = def_period*vr/dc;
    
    l_params(2) = 2.5*abs(l_params(1))/8 + 1;
    r_params(2) = 2.5*abs(r_params(1))/8 + 1;
    
    l_params(3) = def_period;
    r_params(3) = def_period;
    
    paramTraj = [l_params, r_params];
    
end