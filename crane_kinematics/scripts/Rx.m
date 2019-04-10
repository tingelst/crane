function [ Rx1 ] = Rx(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Rx1 = [1,0,0;
        0,cos(x),-sin(x);
        0,sin(x),cos(x)];

end
