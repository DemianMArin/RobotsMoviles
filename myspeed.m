function [speed,maxspeed] = myspeed(xdot,ydot)
   speed = [];
   for i=1:length(xdot)
       speed = [speed sqrt( xdot(i)^2 + ydot(i)^2)];
   end
   maxspeed = max(speed);
end
