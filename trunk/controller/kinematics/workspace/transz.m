function [ Tz ] = transz( d )
%TRANSZ Summary of this function goes here
%   Detailed explanation goes here
Tz = [ 1 0 0 0;
       0 1 0 0;
       0 0 1 sym(d);
       0 0 0 1];
end