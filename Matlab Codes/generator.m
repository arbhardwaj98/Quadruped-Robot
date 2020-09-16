format long

% ..... Geometrical .....
r1 = 10; % length of Cranks
r2 = 20; % length of Coupler Links
L = 5; % Distance between the pivot points
base_l = 45; % Base length
base_w = 25; % Base width
d = 0; % Offset distance between side edge and foot center

% ..... Trajectory .....
stroke = 12; % Distance covered during stance
stroke_h = 18; % 
ht = 6; % Step height

% ..... Temporal .....
period = 1; 
dc = 0.8; % Duty factor
dt = 0.025; % Extension time
phi1 = +0.5; % Phase diff. b/w Contralateral legs (Leg 1 and Leg 2,Leg 2 and Leg 4)
phi2 = +0.75; % Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3,Leg 2 and Leg 4)
w = 2*pi/period;

xc1 = L/2; yc1 = 0; % Joint-1 coordinates
xc2 = -L/2; yc2 = 0; % Joint-2 coordinates

a3 = (-4*(stroke/2+stroke/dc*dt)-2*stroke/dc*(period-dc-2*dt))/(period-dc-2*dt)^3;
a2 = -3/2*a3*(period-dc-2*dt);
a1 = -stroke/dc;
a0 = -(stroke/2+stroke/dc*dt);

i=1;
step = 0.01;
n = 1/step + 1;

Time = zeros(n, 1);
x = zeros(n, 1); y = zeros(n, 1);
Q1 = zeros(n, 1); Q2 = zeros(n, 1); o1 = zeros(n, 1); o2 = zeros(n, 1);
a = zeros(n, 1); b = zeros(n, 1);

for time = 0:step:1
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

    a(i) = sqrt((x(i) - xc1)^2 + (y(i) - yc1)^2);
    b(i) = sqrt((x(i) - xc2)^2 + (y(i) - yc2)^2);
    o1(i) = acos((-r2^2 + r1^2 + b(i)^2 )/(2*r1*b(i)));
    o2(i) = acos((-r2^2 + r1^2 + a(i)^2 )/(2*r1*a(i)));
    g1 = acos (-(a(i)^2 - L^2 - b(i)^2 )/(2*L*b(i)));
    g2 = acos (-(b(i)^2 - L^2 - a(i)^2 )/(2*L*a(i)));
    Q2(i) = - (g1+o1(i))*180/pi;
    Q1(i) = - (pi-(g2+o2(i)))*180/pi;
    %xa = xc1 + r1 * cos(Q1(i));
    %ya = yc1 + r1 * sin(Q1(i));
    %xb = xc2 + r1 * cos(Q2(i));
    %yb = yc2 + r1 * sin(Q2(i));
    
    i=i+1;
end

plot(x, y);

leg1 = [base_l/2 , base_w/2];
leg2 = [base_l/2 , -base_w/2];
leg3 = [-base_l/2 , base_w/2];
leg4 = [-base_l/2 , -base_w/2];
