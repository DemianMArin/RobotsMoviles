clc
clear
syms r theta d L

M = [(1/2)*r*cos(theta)+((d*r)/(2*L))*sin(theta), (1/2)*r*cos(theta)-((d*r)/(2*L))*sin(theta)
     (1/2)*r*sin(theta)-((d*r)/(2*L))*cos(theta), (1/2)*r*sin(theta)+((d*r)/(2*L))*cos(theta)];

 inv_M = inv(M);
 
 latex(inv_M)

 disp(inv_M)
