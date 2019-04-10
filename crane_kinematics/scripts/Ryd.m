function [ Ryd1 ] = Ryd(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Ryd1 = [cosd(x), 0, sind(x);
      0,1,0;
      -sind(x),0,cosd(x)];

end
