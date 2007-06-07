function [ Tx ] = transx( d )
%TRANSZ Summary of this function goes here
%   Detailed explanation goes here
Tx = [ 1 0 0 sym(d);
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
end