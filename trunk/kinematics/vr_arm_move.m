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

for i=1:20
    pos=[0; 1.2+i/10;  3.3];
    dir=0;
    theta03 = arm_inverse(pos,dir);
    vr_arm_init(myworld,[theta03]);
    myworld.inverseball.translation = [pos(2), pos(3), pos(1)];
        myworld.inverseball.rotation    = [0, 1, 0,dir];
    vrdrawnow;
    %pause(0.1);
end
pause(1);

for i=1:15
    pos=[i/15; 3.2;  3.3];
    dir=0;
    theta03 = arm_inverse(pos,dir);
    vr_arm_init(myworld,[theta03]);
    myworld.inverseball.translation = [pos(2), pos(3), pos(1)];
        myworld.inverseball.rotation    = [0, 1, 0,dir];
    vrdrawnow;
    %pause(0.1);
end
pause(1);

for i=1:30
    pos=[ 1; 3.2; 3.3];
    dir=i/60;
    theta03 = arm_inverse(pos,dir);
    vr_arm_init(myworld,[theta03]);
    myworld.inverseball.translation = [pos(2), pos(3), pos(1)];
    myworld.inverseball.rotation    = [0, 1, 0,dir];
    vrdrawnow;
    %pause(0.1);
end
pause(1);
for i=1:30
    pos=[(31-i)/30; 1.2+(31-i)/15;  3.3];
    dir=(31-i)/60;
    theta03 = arm_inverse(pos,dir);
    vr_arm_init(myworld,[theta03]);
    myworld.inverseball.translation = [pos(2), pos(3), pos(1)];
        myworld.inverseball.rotation    = [0, 1, 0,dir];
    vrdrawnow;
    %pause(0.1);
end