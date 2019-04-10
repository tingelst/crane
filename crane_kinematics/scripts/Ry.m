function [ Ry1 ] = Ry(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Ry1 = [cos(x), 0, sin(x);
      0,1,0;
      -sin(x),0,cos(x)];

end
