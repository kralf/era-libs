function [ theta ] = arm_inverse( t_pos, t_angle )
% Semester Project: Embedded Robot Arm Position Controll
% Author: Fritz Stöckli
% 
% Calculate Inverse kinematics

% Arm parameters
a3=2.305;
ap=2.24;
a4=1.88;


t_pos= t_pos - [ 0; 0; a3+a4+ap];
t_direction = [-sin(t_angle); cos(t_angle); 0];
n = [t_pos(2)*t_direction(3) - t_pos(3)*t_direction(2);
     t_pos(3)*t_direction(1) - t_pos(1)*t_direction(3);
     t_pos(1)*t_direction(2) - t_pos(2)*t_direction(1)];
%n = n*sign(n(1));
theta(1) = atan(n(2)/n(1));
theta(2) = atan( n(3)/sqrt((n(1)^2 + n(2)^2))   );

x = t_pos'*[ sin(theta(2))*cos(theta(1)); sin(theta(2))*sin(theta(1)); -cos(theta(2)) ];
y = t_pos'*[ -sin(theta(1)); cos(theta(1)); 0 ];

c4 = (x^2+y^2- a3^2 - ap^2) / (2*ap*a3);
theta(4) = atan(sqrt(1-c4^2)/c4);
theta(3) = atan(y/x) - atan( (ap*sqrt(1-c4^2)) / (a3+ap*c4) );


theta(5)= pi/2 -theta(3)-theta(4);
theta(6)=theta(2);

end