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

% Start and Finish times
t0 = [ 0, 25, 65, 75, 86, 100];
tf = [ 25, 65, 75, 86, 100, 120];
% Start and Finish pos and velStart and Finish pos and vel
xd0 = [    0, 0;   1, 1.5;  3, 0;   3, 0; 3.8, 0; 4,1]; 
xdf = [  1, 1.5;     3, 0;  3, 0; 3.8, 0;   4, 1; 4,1];
vxd0 = [    0, 0;   0.06, 0.06;  0.0, 0.0; 0.0, 0.0; 0.06, 0.06; 0.0,0.0]; 
vxdf = [0.06, 0.06;   0.0, 0.0;  0.0, 0.0; 0.06, 0.06; 0.0, 0.0; 0.0,0.0];  

%Calculating coefficients and time
coefsx = [];
coefsy = [];
N = 1000;
time = [];
for i= 1:6
   coefsx = [coefsx mypolcoefs( t0(i), tf(i),  xd0(i,1),  vxd0(i,1),  xdf(i,1), vxdf(i,1)) ];
   coefsy = [coefsy mypolcoefs( t0(i), tf(i),  xd0(i,2),  vxd0(i,2),  xdf(i,2), vxdf(i,2)) ];
   time = [time, linspace(t0(i),tf(i),N)];
end 

vpa(coefsx, 6)
vpa(coefsy, 6)

% Caculating 4 polynomials + (one stationary)
x = [];
xdot = [];
y = [];
ydot = [];
speed = [];

for i=1:6
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


% Drawing trayectory with radius
plot(x,y,color='blue')
title('Trajectory')
hold on
for i=1:1:length(x)
    rectangle('Position',[x(i)-0.1,y(i)-0.1,0.2,0.2],'Curvature',[1 1])
    hold on
end

% Plotting speed
figure
plot(time, speed)
hold on
plot([0 100], [0.1 0.1])
title('Speed')

vpa(x(1,1:10),5)
vpa(y(1,1:10),5)




