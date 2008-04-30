% Semester Project: Embedded Robot Arm Position Controll
% Author: Fritz Stöckli
%
% Kinematics Simulation
% 
% Usage: (Virtual Reality Toolbox needed)
% - run vr_arm_start.m   (-> vr-window opens)
% - run vr_arm_move.m    (-> arm follows trajectory)



clear all

% Open Virtual Reality Scene

myworld = vrworld('vr_arm_scene.wrl') 
open(myworld);  
f = vrfigure(myworld);
set(f,'CameraPosition', [11.16 7.12 -7.67])
set(f,'CameraDirection', [-0.74 -0.29 0.61])


% Initiate starting position

% calculate inverse kinematics
theta = arm_inverse([ 0; 1.3; 3.3  ],0);
vr_arm_init(myworld,[theta]);
% place the yellow arrow at the same position
myworld.inverseball.translation = [1.3, 3.3, 0];
vrdrawnow;