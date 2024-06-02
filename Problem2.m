dbstop if error
clear all;
clc;
close all;

% Start and Finish times
global tf
t0 = [ 0, 25, 65, 75, 86];
tf = [ 25, 65, 75, 86, 100];
% Start and Finish pos and velStart and Finish pos and vel
xd0 = [    0, 0;   1, 1.5;  3, 0;   3, 0; 3.8, 0]; 
xdf = [  1, 1.5;     3, 0;  3, 0; 3.8, 0;   4, 1];
vxd0 = [    0, 0;   0.06, 0.06;  0.0, 0.0; 0.0, 0.0; 0.06, 0.06]; 
vxdf = [0.06, 0.06;   0.0, 0.0;  0.0, 0.0; 0.06, 0.06; 0.0, 0.0];  

%%% Trajectory
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


%%% Solving for state
global ex 
global ey
ex = [];
ey = [];
% SET THE CORRECT INITIAL CONDITIONS
IC = [0;1.5];  %[x10;x20]
[t,state] = ode45(@(t,state) mysolver(t, state), time, IC);

%%% Plotting
% Path
figure
x1 = state(:,1);
x2 = state(:,2);
plot(x1,x2)
title('x1 vs x2')


%DEFINE YOUR CONTROL LAW HERE!!!
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

%DO NOT TOUCH:
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
