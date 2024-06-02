dbstop if error 
clc;
close all;
clear all; %#ok<CLALL> 

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


%%% Control
% Normal
IC = [0;1.5];  %[x10;x20]
[t,state] = ode45(@(t,state) mysolver(t, state), time, IC);


%%% Plotting
% Plotting trayectory 
plot(x,y,color='blue')
hold on
for i=1:1:length(x)
    rectangle('Position',[x(i)-0.1,y(i)-0.1,0.2,0.2],'Curvature',[1 1])
    hold on
end
% Plotting control robot
x1 = state(:,1);
x2 = state(:,2);
plot(x1,x2)
title('Robot and Path')
% Plotting mag error vs time
figure
ex = x1'-x;
ey = x2'-y;
mag = sqrt( ex.^2 + ey.^2);
plot(time, mag)  
title('|Error|')
% Plotting speed
%figure
%plot(time, speed)
%title('Speed vs time')
% Plotting u vs time
u1 = [];
u2 = [];
for index = 1:1:length(time)
    u  = cont(x1(index,1),x2(index,1),time(index));
    u1 = [u1 u(1,1)];
    u2 = [u2 u(2,1)];
end
figure 
plot(time, u1)
title('U1 vs time')
figure
plot(time, u2)
title('U2 vs time')

%%% Functions
% Control 
function u = cont(x1,x2,t)
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

    u1 = -0.2*(x1-xd) + xdd;
    u2 = -0.2*(x2-yd) + ydd;
    u = [u1;u2];
end

% Differential equation
function dstatedt = mysolver(t, state)
 
    x1 = state(1,1);
    x2 = state(2,1);

    % The controller u
        u = cont(x1,x2,t); %xdot = f(x), donde u=f(x)
        u1 = u(1,1);
        u2 = u(2,1);

    % The differential equation 
        xdot = [u1;u2];

    dstatedt = xdot;     
 end














