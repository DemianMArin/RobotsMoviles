function [xd,xdd] = myindpol(t,t0,coefs)
    xd = coefs(1,1) + coefs(2,1)*(t-t0) + coefs(3,1)*(t-t0)^2 + coefs(4,1)*(t-t0)^3;
    xdd = 0 + coefs(2,1) + 2*coefs(3,1)*(t-t0) + 3*coefs(4,1)*(t-t0)^2;
end
