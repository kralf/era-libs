
clc
starting_karth = [ 0, 45, 20, 0];
%starting_karth = [40, 25, 35, -30/180*pi];

    a_3=23.05;
    a_4=22.4;
    a_5=18.8;
    a = a_3+ a_4 + a_5;

x= starting_karth(1);

y= starting_karth(2);

z= starting_karth(3)+18.8;

beta_1= starting_karth(4);

beta_2= 0

theta_1 = beta_1

theta_2 = atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )

theta_3 = atan( (  -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) ) / ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) ) )  -  atan( ( a_4 * sqrt( 1 - ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) )^2 ) ) / (a_3 + a_4* ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) ) ) )

theta_4 = atan( sqrt(1 - ( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) )^2 ) /( ( ( sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*cos(beta_1) *(x+a_5*sin(beta_1)) + sin(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ))*sin(beta_1) *(y-a_5*cos(beta_1))  - cos(atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) )) * (z-a) )^2 + ( -sin(beta_1) *(x+a_5*sin(beta_1))  + cos(beta_1) *(y-a_5*cos(beta_1)) )^2 - a_3^2 - a_4^2 ) / (2*a_3*a_4) ) )

theta_6 = atan( ( (x+a_5*sin(beta_1))*cos(beta_1) - (y-a_5*cos(beta_1))*(-sin(beta_1)) ) / sqrt((z-a)^2) ) + beta_2



                pos6 = [
                     (-((-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4)*a_4+cos(theta_1)*sin(theta_2)*cos(theta_3)*a_3-sin(theta_1)*sin(theta_3)*a_3;
                     (-((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4)*a_4+sin(theta_1)*sin(theta_2)*cos(theta_3)*a_3+cos(theta_1)*sin(theta_3)*a_3;                                                                                                                                                                                                                                                                     
                     (-(cos(theta_2)*cos(theta_3)*cos(theta_4)-cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5-cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4+cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4-cos(theta_2)*cos(theta_3)*a_3+a_3+a_4+a_5;
                     ];
