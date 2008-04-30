function [ output_args ] = forward_init( myworld, theta )
% Semester Project: Embedded Robot Arm Position Controll
% Author: Fritz Stöckli
%
% Kinematics Simulation
% 
% Usage: (Virtual Reality Toolbox needed)
% - run vr_arm_start.m   (-> vr-window opens)
% - run vr_arm_move.m    (-> arm follows trajectory)


a3=2.305;
a4=2.24;
a5=1.88;

%H15;

myworld.cs1.rotation      = [0 1 0 theta(1)];      %z:theta
myworld.cs1.translation   = [0  a3+a4+a5 0];       %z:d
%myworld.cs1_2.translation = [0  0 0];             %x:a
myworld.cs1_2.rotation    = [0 0 1 pi/2];          %x:alpha

myworld.cs2.rotation      = [0 1 0 theta(2)+pi/2];
myworld.cs2.translation   = [0  0 0];
%myworld.cs2_2.translation = [0  0 0];
myworld.cs2_2.rotation    = [0 0 1 pi/2];

myworld.cs3.rotation = [0 1 0 theta(3)];
%myworld.cs3.rotation    = [0 0 0 0];
myworld.cs3_2.translation = [0  0 -a3];
%myworld.cs3_2.translation = [0  0 0];


myworld.cs4.rotation = [0 1 0 theta(4)];
myworld.cs4_2.translation = [0  0 -a4];

myworld.cs5.rotation = [0 1 0 theta(5)-pi/2];
myworld.cs5_2.rotation = [0 0 1 pi/2];

myworld.cs6.rotation      = [0 1 0 theta(6)];      %z:theta
myworld.cs6.translation   = [0  a5 0];       %z:d
%myworld.cs6_2.translation = [0  0 0];             %x:a
%myworld.cs6_2.rotation    = [0 0 1 pi/2];          %x:alpha

end