function [ Rzd1 ] = Rzd(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Rzd1 = [cosd(x),-sind(x),0;
      sind(x),cosd(x),0;
      0,0,1];

end