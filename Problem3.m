dbstop if error
clear all;
clc;
close all;

patchlowx = [-1, -1, 1, 1, 2, 2, 5, 5];
patchlowy = [-1.5, -1, -1, 1, 1, -1, -1, -1.5];

patchhighx = [-1, -1, 3, 3, 3.5, 3.5, 5, 5];
patchhighy = [3, 2, 2, 0.5, 0.5, 2, 2, 3];


figure
hold on
axis equal
patch(patchlowx,patchlowy,'red')
patch(patchhighx,patchhighy,'red')
grid on
grid minor


%%% Trajectory
% Start and Finish times
global tf
t0 = [ 0, 25, 65, 75, 86];
tf = [ 25, 65, 75, 86, 100];
% Start and Finish pos and velStart and Finish pos and vel
xd0 = [    0, 0;   1, 1.5;  3, 0;   3, 0; 3.8, 0]; 
xdf = [  1, 1.5;     3, 0;  3, 0; 3.8, 0;   4, 1];
vxd0 = [    0, 0;   0.06, 0.06;  0.0, 0.0; 0.0, 0.0; 0.06, 0.06]; 
vxdf = [0.06, 0.06;   0.0, 0.0;  0.0, 0.0; 0.06, 0.06; 0.0, 0.0];  

%Calculating coefficients and time
global coefsx
global coefsy
coefsx = [];
coefsy = [];
N = 1000;
time = [];
for i= 1:5
   coefsx = [coefsx mypolcoefs( t0(i), tf(i),  xd0(i,1),  vxd0(i,1),  xdf(i,1), vxdf(i,1)) ];
   coefsy = [coefsy mypolcoefs( t0(i), tf(i),  xd0(i,2),  vxd0(i,2),  xdf(i,2), vxdf(i,2)) ];
   time = [time, linspace(t0(i),tf(i),N)];
end 

% Caculating 4 polynomials + (one stationary)
x = [];
xdot = [];
y = [];
ydot = [];
speed = [];

for i=1:5
   x_temp = [];
   xdot_temp = [];
   y_temp = [];
   ydot_temp = [];
   speed_temp = [];

   if i == 1
       [x_temp xdot_temp] = mypol(t0(i), coefsx(:,i), time(1,1:1000));
       [y_temp ydot_temp] = mypol(t0(i), coefsy(:,i), time(1,1:1000));
       [speed_temp maxspeed] = myspeed(xdot_temp,ydot_temp);
   else
       [x_temp xdot_temp] = mypol(t0(i), coefsx(:,i), time(1,(i-1)*1000 +1:i*1000));
       [y_temp ydot_temp] = mypol(t0(i), coefsy(:,i), time(1,(i-1)*1000 +1:i*1000));
       [speed_temp maxspeed] = myspeed(xdot_temp,ydot_temp);
   end
   x = [x x_temp];
   xdot = [xdot xdot_temp];
   y = [y y_temp];
   ydot = [ydot ydot_temp];
   speed = [speed speed_temp];
end

% Initial Conditions 
IC = [0;1.5;0];  %[x10;x20;theta0]

%DO NOT TOUCH:
r = 0.05;
l = 0.1;
d = 0.05;
[t,state] = ode45(@(t,state) mysolver(t, state, r, l, d), time, IC);


%%% Plotting 
% Plotting trayectory 
plot(x,y,color='blue')
hold on
for i=1:1:length(x)
    rectangle('Position',[x(i)-0.1,y(i)-0.1,0.2,0.2],'Curvature',[1 1])
    hold on
end
% Robot state
plot(state(:,1),state(:,2),'color','blue')
% Plotting mag error vs time
figure
x1 = state(:,1);
x2 = state(:,2);
x3 = state(:,3);
ex = x1'-x;
ey = x2'-y;
mag = sqrt( ex.^2 + ey.^2);
plot(time, mag)  
title('|Error|')
% Plotting theta
figure
plot(time,x3);
title('Theta vs time')
% Plotting omega 
u1 = [];
u2 = [];
for index = 1:1:length(time)
    u  = wheelvel(x1(index,1),x2(index,1),x3(index,1),r,l,d,time(index));
    u1 = [u1 u(1,1)];
    u2 = [u2 u(2,1)];
end
figure
plot(time,u1);
title('omega1 vs time')
figure
plot(time,u2);
title('omega2 vs time')

%DEFINE YOUR CONTROL LAW HERE!!!
function omega = wheelvel(x1,x2,theta,r,l,d,t)
    global tf
    global coefsx
    global coefsy
    xd = [];
    xdd = [];
    yd = [];
    ydd = [];

    if t<=tf(1) 
        [xd, xdd] = myindpol(t, 0, coefsx(:,1));
        [yd, ydd] = myindpol(t, 0, coefsy(:,1));
    elseif t>tf(1) && t<=tf(2)
        [xd,xdd] = myindpol(t, tf(1), coefsx(:,2));
        [yd,ydd] = myindpol(t, tf(1), coefsy(:,2));
    elseif t>tf(2) && t<=tf(3)
        [xd,xdd] = myindpol(t, tf(2), coefsx(:,3));
        [yd,ydd] = myindpol(t, tf(2), coefsy(:,3));
    elseif t>tf(3) && t<=tf(4)
        [xd,xdd] = myindpol(t, tf(3), coefsx(:,4));
        [yd,ydd] = myindpol(t, tf(3), coefsy(:,4));
    elseif t>tf(4) && t<=tf(5)
        [xd,xdd] = myindpol(t, tf(4), coefsx(:,5));
        [yd,ydd] = myindpol(t, tf(4), coefsy(:,5));
    end

    u1 = -0.2 * (x1 - xd) + xdd;
    u2 = -0.2 * (x2 - yd) + ydd;

    inv_M = [(l*sin(theta)+d*cos(theta)) / (d*r*cos(theta)^2 + d*r*sin(theta)^2), -1*((l*cos(theta)-d*sin(theta)) / (d*r*cos(theta)^2 + d*r*sin(theta)^2));
            -1*((l*sin(theta)-d*cos(theta)) / (d*r*cos(theta)^2 + d*r*sin(theta)^2)), (l*cos(theta)+d*sin(theta)) / (d*r*cos(theta)^2 + d*r*sin(theta)^2)];

    omega  = inv_M*[u1;u2];
end

%DO NOT TOUCH:
function dstatedt = mysolver(t, state, r, l, d)
 
    x1 = state(1,1);
    x2 = state(2,1);
    theta = state(3,1);

    % Wheel velocities
        omega = wheelvel(x1,x2,theta,r,l,d,t); 
        omega1 = omega(1,1);
        omega2 = omega(2,1);

    % The differential equation 
        x1dot = ((r/2)*cos(theta) + (d*r/(2*l))*sin(theta))*omega1 +...
                ((r/2)*cos(theta) - (d*r/(2*l))*sin(theta))*omega2;
        x2dot = ((r/2)*sin(theta) - (d*r/(2*l))*cos(theta))*omega1 +...
                ((r/2)*sin(theta) + (d*r/(2*l))*cos(theta))*omega2;
        thetadot = -(r/(2*l))*omega1 + (r/(2*l))*omega2;

    dstatedt = [x1dot;x2dot;thetadot];     
     
    %display(t)
 end
