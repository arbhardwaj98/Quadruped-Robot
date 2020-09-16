function min_margin = checkStability(x_l, y_l, x_r, y_r, animate)  

    load('quad_params.mat')
    
    leg1 = [base_l/2 , 0, base_w/2];
    leg2 = [base_l/2 , 0, -base_w/2];
    leg3 = [-base_l/2 , 0, base_w/2];
    leg4 = [-base_l/2 , 0, -base_w/2];

    z = zeros(size(x_l));
    n = length(x_l);
    step_n = 1/n;
    margin = (2*base_l)*ones(size(x_l));
    cord_l = [x_l, y_l, z];
    cord_r = [x_r, y_r, z];
    p = zeros(4,3);
    ix = 1;
    phase = [0, 0.5, 0.25, 0.75];
    for time = 0:step_n:(1-step_n)
        p(1,:) = leg1+cord_l(round(mod((phase(1) + time)/step_n,n)+1),:);
        p(2,:) = leg2+cord_l(round(mod((phase(2) + time)/step_n,n)+1),:);
        p(3,:) = leg3+cord_r(round(mod((phase(3) + time)/step_n,n)+1),:);
        p(4,:) = leg4+cord_r(round(mod((phase(4) + time)/step_n,n)+1),:);
        gd = [];
       for i = 1:4
           if abs(p(i,2)+stroke_h) < 0.001
               gd = [gd; p(i,:)];
           end
       end
       if length(gd) == 4
        time;

       else
           pt = [0,0];
           [val1, d1] = signPoints(pt, gd(1,[1,3]), gd(2,[1,3]));
           [val2, d2] = signPoints(pt, gd(2,[1,3]), gd(3,[1,3]));
           [val3, d3] = signPoints(pt, gd(3,[1,3]), gd(1,[1,3]));

           has_neg = (val1 < 0) || (val2 < 0) || (val3 < 0);
           has_pos = (val1 > 0) || (val2 > 0) || (val3 > 0);
           stable = has_neg || has_pos;

           if not(stable)
               disp("ROBOT IS UNSTABLE");
               margin(ix) = -1;
           else
               margin(ix) = min([d1, d2, d3]);
           end
       end
       
       ix=ix+1;
       
       if animate
           pgon = convhull(gd(:,1),gd(:,3));
           plot(gd(pgon,1),gd(pgon,3),'LineWidth', 3);
           xlim([-(0.65*base_l+5), 0.65*base_l+5])
           ylim([-(base_w/2+5), base_w/2+5])
           hold on
           plot(0,0,"or", 'MarkerSize', 7 , 'MarkerFaceColor', 'r');
           pause(step_size)
           hold off
       end
    end
    min_margin = min(margin);
    
end

