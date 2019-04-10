function [ Rxd1 ] = Rxd(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Rxd1 = [1,0,0;
        0,cosd(x),-sind(x);
        0,sind(x),cosd(x)];

end
