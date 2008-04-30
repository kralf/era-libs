clc
if(0)
    syms  x y z beta_1 beta_2 a_3 a_4 a_5 a
else
    x = 10;
    y = 28.3;
    z = 35.7;
    beta_1 = -0.2;
    beta_2 = 0.2;
    
    a_3=23.05;
    a_4=22.4;
    a_5=18.8;
    a = a_3+ a_4 + a_5;
    
    a3=23.05;
    ap=22.4;
    a4=18.8;

    t_pos = [x;y;z];
    t_angle = beta_1;
end

theta_1 = beta_1

theta_2 = atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )


%xe =  sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a);

%ye =  -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) ;

%co4 = ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4)


theta_3 = atan( (  -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) ) / ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) ) )  -  atan( ( a_4 * sqrt( 1 - ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) )^2 ) ) / (a_3 + a_4* ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) ) ) )

theta_4 = atan( sqrt(1 - ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) )^2 ) /( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) ) )


theta_6 = atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ) + beta_2

t_pos= t_pos - [ 0; 0; a3+a4+ap] + [a4*sin(t_angle); -a4*cos(t_angle); 0];

t_direction = [-sin(t_angle); cos(t_angle); 0];
n = [t_pos(2)*t_direction(3) - t_pos(3)*t_direction(2);
     t_pos(3)*t_direction(1) - t_pos(1)*t_direction(3);
     t_pos(1)*t_direction(2) - t_pos(2)*t_direction(1)];
%n = n*sign(n(1));
theta(1) = t_angle  ; %  atan(n(2)/n(1))
theta(2) =   atan( n(3)/sqrt((n(1)^2 + n(2)^2))   )

x = t_pos'*[ sin(theta(2))*cos(theta(1)); sin(theta(2))*sin(theta(1)); -cos(theta(2)) ];
y = t_pos'*[ -sin(theta(1)); cos(theta(1)); 0 ];

c4 = (x^2+y^2- a3^2 - ap^2) / (2*ap*a3)
theta(4) = atan(sqrt(1-c4^2)/c4)
theta(3) = atan(y/x) - atan( (ap*sqrt(1-c4^2)) / (a3+ap*c4) );


theta(5)= pi/2 -theta(3)-theta(4);
theta(6)=theta(2);