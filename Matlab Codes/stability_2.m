format long

% ..... Geometrical .....
r1 = 10;                % length of Cranks
r2 = 20;                % length of Coupler Links
L = 5;                  % Distance between the pivot points
base_l = 25;            % Base length
base_w = 25;            % Base width
d = 0;                  % Offset distance between side edge and foot center

% ..... Trajectory .....

lin_vel = 0;
ang_vel = 0;

stroke = 12;            % Distance covered during stance
stroke_h = 18;          % Robot Height
ht = 6;                 % Step height

% ..... Temporal .....
period = 1; 
dc = 0.8;               % Duty factor
dt = 0.03*period;       % Extension time
phi1 = +0.5;            % Phase diff. b/w Contralateral legs (Leg 1 and Leg 4, Leg 2 and Leg 3)
phi2 = +0.75;           % Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3, Leg 2 and Leg 4)
w = 2*pi/period;

%..................................%

xc1 = L/2; yc1 = 0;     % Joint-1 coordinates
xc2 = -L/2; yc2 = 0;    % Joint-2 coordinates

C_x = [          - (stroke/2) - (dt*stroke)/(dc*period);
                                   - stroke/(dc*period);
          (3*stroke)/(dc*(2*dt - period + dc*period)^2);
         (2*stroke)/(dc*(2*dt - period + dc*period)^3)];
   
C_y = [                       -stroke_h;
                                      0;
          (16*ht)/(period^2*(dc - 1)^2);
          (32*ht)/(period^3*(dc - 1)^3);
         (16*ht)/(period^4*(dc - 1)^4)];

step = 0.001;
n = 1/step;

Time = (0:step:(period-step))';
x = zeros(n, 1); y = zeros(n, 1); x2 = zeros(n, 1); y2 = zeros(n, 1);

%.......... X vs Time ..........%
Phase = [((dc*period+dt)/step); ((period-dt)/step)];
x(1:Phase(1)) = stroke/2 - stroke*Time(1:Phase(1))/(dc*period);
t = Time(Phase(1)+1:Phase(2)) - (dc*period+dt);
x(Phase(1)+1:Phase(2)) = [ones(size(t,1),1), t, t.^2, t.^3]*C_x;
x(Phase(2)+1:end) = stroke/2 + stroke*(period - Time(Phase(2)+1:end))/(dc*period);

%.......... Y vs Time ..........%
y(1:((dc*period)/step)) = -1*stroke_h*ones(((dc*period)/step),1);
t = Time(((dc*period)/step+1):end) - (dc*period);
y(((dc*period)/step+1):end) = [ones(size(t,1),1), t, t.^2, t.^3, t.^4]*C_y;


% plot(Time, x);
% w = waitforbuttonpress;
% plot(Time, y);
% w = waitforbuttonpress;
% plot(x,y);
% 
% Q1 = zeros(n, 1); Q2 = zeros(n, 1); o1 = zeros(n, 1); o2 = zeros(n, 1);
% a = zeros(n, 1); b = zeros(n, 1);

a = sqrt((x - xc1).^2 + (y - yc1).^2);
b = sqrt((x - xc2).^2 + (y - yc2).^2);
o1 = acos((-r2^2 + r1^2 + b.^2 )./(2*r1*b));
o2 = acos((-r2^2 + r1^2 + a.^2 )./(2*r1*a));
g1 = acos (-(a.^2 - L^2 - b.^2 )./(2*L*b));
g2 = acos (-(b.^2 - L^2 - a.^2 )./(2*L*a));
Q2 = -(g1+o1)*180/pi;
Q1 = -(pi-(g2+o2))*180/pi;

toc
    
xa = xc1 + r1 * cosd(Q1);
ya = yc1 + r1 * sind(Q1);
xb = xc2 + r1 * cosd(Q2);
yb = yc2 + r1 * sind(Q2);

for i = 1:length(x)
    figure(1)
    plot([xc2,xb(i),x(i),xa(i),xc1],[yc2,yb(i),y(i),ya(i),yc1]);

    xlim([-15,15]);
    ylim([-23, 7]);
    pause(step)
end
   
    
% xlim([-15,15]);
% ylim([-23, 7]);
% hold ;
% plot([xc1,xc2],[yc1,yc2],'-or','MarkerSize', 7 , 'MarkerFaceColor', 'r');
% hold on
% plot([xc2,xb,x(41),xa,xc1],[yc2,yb,y(41),ya,yc1]);
% hold on;
% plot([x;x(1)], [y;y(1)]);

% leg1 = [base_l/2 , 0, base_w/2];
% leg2 = [base_l/2 , 0, -base_w/2];
% leg3 = [-base_l/2 , 0, base_w/2];
% leg4 = [-base_l/2 , 0, -base_w/2];
% 
% z = zeros(size(x));
% margin = (base_l+stroke)*ones(size(x));
% cord = [x, y, z];
% phase = [0.0, 0.5, 0.25, 0.75];
% p = zeros(4,3);
% ix = 1;
% for time = 0:step:(1-step)
%    time
%    p(1,:) = leg1+cord(round(mod((phase(1) + time)/step,n)+1),:);
%    p(2,:) = leg2+cord(round(mod((phase(2) + time)/step,n)+1),:);
%    p(3,:) = leg3+cord(round(mod((phase(3) + time)/step,n)+1),:);
%    p(4,:) = leg4+cord(round(mod((phase(4) + time)/step,n)+1),:);
%    gd = [];
%    for i = 1:4
%        if abs(p(i,2)+stroke_h) < 0.001
%            gd = [gd; p(i,:)];
%        end
%    end
%    if length(gd) == 4
%        
%    else
%        pt = [0,0];
%        [val1, d1] = signPoints(pt, gd(1,[1,3]), gd(2,[1,3]));
%        [val2, d2] = signPoints(pt, gd(2,[1,3]), gd(3,[1,3]));
%        [val3, d3] = signPoints(pt, gd(3,[1,3]), gd(1,[1,3]));
% 
%        has_neg = (val1 < 0) || (val2 < 0) || (val3 < 0);
%        has_pos = (val1 > 0) || (val2 > 0) || (val3 > 0);
%        stable = has_neg || has_pos;
%    
%        if not(stable)
%            disp("ROBOT IS UNSTABLE");
% %            break;
%        else
%            margin(ix) = min([d1, d2, d3]);
%        end
%    end
%    ix=ix+1;
%    pgon = convhull(gd(:,1),gd(:,3));
%    plot(gd(pgon,1),gd(pgon,3),'LineWidth', 3);
%    xlim([-(base_l/2+stroke+5), base_l/2+stroke+5])
%    ylim([-(base_w/2+5), base_w/2+5])
%    hold on
%    plot(0,0,"or", 'MarkerSize', 7 , 'MarkerFaceColor', 'r');
%    pause(0.1)
%    hold off
%    
% end
% disp(min(margin));
% 
% 
