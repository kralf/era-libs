function [ Rz ] = rotz( omega )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here
Rz=[ cos(sym(omega))  -sin(sym(omega)) 0 0;
     sin(sym(omega))   cos(sym(omega)) 0 0;
           0                  0        1 0;
           0                  0        0 1];
end