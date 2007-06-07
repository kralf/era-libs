function [ Rx ] = rotx( omega )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here
Rx=[ 1      0                  0          0
     0  cos(sym(omega))  -sin(sym(omega)) 0;
     0  sin(sym(omega))   cos(sym(omega)) 0;
     0      0                  0          1];
end