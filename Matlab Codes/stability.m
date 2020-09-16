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
dt = 0.025*period;      % Extension time
phi1 = +0.5;            % Phase diff. b/w Contralateral legs (Leg 1 and Leg 4, Leg 2 and Leg 3)
phi2 = +0.75;           % Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3, Leg 2 and Leg 4)
w = 2*pi/period;

%..................................%

xc1 = L/2; yc1 = 0;     % Joint-1 coordinates
xc2 = -L/2; yc2 = 0;    % Joint-2 coordinates

tic
a3 = (-4*(stroke/2+stroke/dc*dt)-2*stroke/dc*(period-dc-2*dt))/(period-dc-2*dt)^3;
a2 = -3/2*a3*(period-dc-2*dt);
a1 = -stroke/dc;
a0 = -(stroke/2+stroke/dc*dt);

i=1;
step = 0.01;
n = 1/step;

Time = zeros(n, 1);
x = zeros(n, 1); y = zeros(n, 1);
Q1 = zeros(n, 1); Q2 = zeros(n, 1); o1 = zeros(n, 1); o2 = zeros(n, 1);
a = zeros(n, 1); b = zeros(n, 1);
%figure(1);

   
for time = 0:step:(1-step)
    Time(i)=time;
    t = mod(time,1);
    if t<=(dc+dt)
        x(i)= stroke/2 + stroke/dc*(-t); %stroke with velocity extension for 'dt'seconds
    elseif t<=(period-dt)
        x(i) = a0 + a1*(t-(dc+dt)) + a2*(t-(dc+dt))^2 + a3*(t-(dc+dt))^3;%stride path
    else
        x(i)= stroke/2 + stroke/dc*(period-t);%velocity extension for 'dt'seconds before stride
    end
    
    if t<=dc
        y(i)= -stroke_h;%stance
    else
        y(i) = -stroke_h + 0 + 16*ht/(period-dc)^2*(t-dc)^2 - 32*ht/(period-dc)^3*(t-dc)^3 + 16*ht/(period-dc)^4*(t-dc)^4;%stride
    end

%     plot(x(i), y(i),"");
%     xlim([-10, 10])
%     ylim([-20, -10])
%     hold on;
%     pause(0.2)
    
    
    a(i) = sqrt((x(i) - xc1)^2 + (y(i) - yc1)^2);
    b(i) = sqrt((x(i) - xc2)^2 + (y(i) - yc2)^2);
    o1(i) = acos((-r2^2 + r1^2 + b(i)^2 )/(2*r1*b(i)));
    o2(i) = acos((-r2^2 + r1^2 + a(i)^2 )/(2*r1*a(i)));
    g1 = acos (-(a(i)^2 - L^2 - b(i)^2 )/(2*L*b(i)));
    g2 = acos (-(b(i)^2 - L^2 - a(i)^2 )/(2*L*a(i)));
    Q2(i) = - (g1+o1(i))*180/pi;
    Q1(i) = - (pi-(g2+o2(i)))*180/pi;
    
    
    
%     xa = xc1 + r1 * cosd(Q1(i));
%     ya = yc1 + r1 * sind(Q1(i));
%     xb = xc2 + r1 * cosd(Q2(i));
%     yb = yc2 + r1 * sind(Q2(i));
    
%     figure(1)
%     plot([xc2,xb,x(i),xa,xc1],[yc2,yb,y(i),ya,yc1]);
%     
%     xlim([-15,15]);
%     ylim([-23, 7]);
%     pause(step)
    
    i=i+1;
end

toc

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
