function [coeffs] = mypolcoefs(t0,tf,xd0,vxd0,xdf,vxdf)
M = [1,     0,         0,           0;  
     0,     1,         0,           0;
     1, tf-t0, (tf-t0)^2,   (tf-t0)^3;
     0,     1, 2*(tf-t0), 3*(tf-t0)^2];

coeffs = M\[xd0; vxd0; xdf; vxdf];
end
