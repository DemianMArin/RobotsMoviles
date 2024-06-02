function [x,xdot] = mypol(t0,coefs,times)
    x = [];
    xdot = [];
    for i=1:length(times)
       x = [x coefs(1) + coefs(2)*(times(i)-t0) + coefs(3)*(times(i)-t0)^2 +   coefs(4)*(times(i)-t0)^3];
       xdot = [xdot      0 +               coefs(2) + 2*coefs(3)*(times(i)-t0) + 3*coefs(4)*(times(i)-t0)^2];
    end
end
