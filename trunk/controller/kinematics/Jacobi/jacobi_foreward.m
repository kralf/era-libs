%clc
analyt =0;

if(analyt)
syms  theta_1 theta_2 theta_3 theta_4 a_3 a_4 a_5 theta_5 theta_6

                pos6 = [
                     (-((-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4)*a_4+cos(theta_1)*sin(theta_2)*cos(theta_3)*a_3-sin(theta_1)*sin(theta_3)*a_3;
                     (-((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4)*a_4+sin(theta_1)*sin(theta_2)*cos(theta_3)*a_3+cos(theta_1)*sin(theta_3)*a_3;                                                                                                                                                                                                                                                                     
                     (-(cos(theta_2)*cos(theta_3)*cos(theta_4)-cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5-cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4+cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4-cos(theta_2)*cos(theta_3)*a_3+a_3+a_4+a_5;
                     ];
                 
%                 latex(simple(diff(pos6(1), theta_1)) )
 
else
    
%    x = 10;
%    y = 28.3;
%    z = 35.7;
%    beta_1 = -0.2;
%    beta_2 = 0.2;
    
    theta_1 =   -0.2000;
    theta_2 =    0.1453;
    theta_3 =   -0.4474;
    theta_4 =   -1.4921;
    theta_6 =    0.3453;
    
    theta_1 =  -0.200000000000000
    theta_2 =   0.145319438709792
    theta_3 =  -0.447407610887534
    theta_4 =  -1.492082716707799
    theta_6 =   0.345319438709792
    

    
    
    a_3=23.05;
    a_4=22.4;
    a_5=18.8;
    %a = a_3+ a_4 + a_5;

end

if(analyt)
    x = diff(pos6(1), theta_1)
    y = diff(pos6(1), theta_2)
    z = diff(pos6(1), theta_3)
    b1= diff(pos6(1), theta_4)
    b2= diff(pos6(1), theta_6)

else
%simple(diff(theta_3, x))

J = zeros(5,5);

J(1,1) = ((-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4)-(-sin(theta_1)*sin(theta_2)*sin(theta_3)+cos(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)+(-sin(theta_1)*sin(theta_2)*sin(theta_3)+cos(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(-sin(theta_1)*sin(theta_2)*sin(theta_3)+cos(theta_1)*cos(theta_3))*sin(theta_4)*a_4-sin(theta_1)*sin(theta_2)*cos(theta_3)*a_3-cos(theta_1)*sin(theta_3)*a_3;
J(1,2) = ((cos(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)-cos(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(cos(theta_1)*cos(theta_2)*cos(theta_3)*sin(theta_4)+cos(theta_1)*cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5+cos(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4-cos(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4+cos(theta_1)*cos(theta_2)*cos(theta_3)*a_3;
J(1,3) = ((-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4))*sin(theta_4+theta_3)+(-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((-cos(theta_1)*sin(theta_2)*sin(theta_3)-sin(theta_1)*cos(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*cos(theta_4))*cos(theta_4+theta_3)+((cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4)*a_4-cos(theta_1)*sin(theta_2)*sin(theta_3)*a_3-sin(theta_1)*cos(theta_3)*a_3;
J(1,4) = (((-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*sin(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3)+(-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)+((cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5+(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*sin(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4)*a_4;
J(1,5) = 0;



J(2,1) = ((-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-((cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4)*a_4+cos(theta_1)*sin(theta_2)*cos(theta_3)*a_3-sin(theta_1)*sin(theta_3)*a_3;
J(2,2) = ((sin(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)-sin(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(sin(theta_1)*cos(theta_2)*cos(theta_3)*sin(theta_4)+sin(theta_1)*cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5+sin(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4-sin(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4+sin(theta_1)*cos(theta_2)*cos(theta_3)*a_3;
J(2,3) = ((-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4))*sin(theta_4+theta_3)+(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((-sin(theta_1)*sin(theta_2)*sin(theta_3)+cos(theta_1)*cos(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4))*cos(theta_4+theta_3)+((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)*a_4-sin(theta_1)*sin(theta_2)*sin(theta_3)*a_3+cos(theta_1)*cos(theta_3)*a_3;
J(2,4) = (((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3)+(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)+((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5+(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)*a_4;
J(2,5) = 0;


   %      ((-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-((cos(theta_1)*sin(theta_2)*cos(theta_3)-sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4)*a_4+cos(theta_1)*sin(theta_2)*cos(theta_3)*a_3-sin(theta_1)*sin(theta_3)*a_3
  %       ((sin(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)-sin(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(sin(theta_1)*cos(theta_2)*cos(theta_3)*sin(theta_4)+sin(theta_1)*cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5+sin(theta_1)*cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4-sin(theta_1)*cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4+sin(theta_1)*cos(theta_2)*cos(theta_3)*a_3
 %        ((-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4))*sin(theta_4+theta_3)+(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((-sin(theta_1)*sin(theta_2)*sin(theta_3)+cos(theta_1)*cos(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4))*cos(theta_4+theta_3)+((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)*a_4-sin(theta_1)*sin(theta_2)*sin(theta_3)*a_3+cos(theta_1)*cos(theta_3)*a_3
%         (((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3)+(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)-((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*cos(theta_4)-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*cos(theta_4+theta_3)+((sin(theta_1)*sin(theta_2)*cos(theta_3)+cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*sin(theta_4+theta_3))*a_5+(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4)*a_4


J(3,1) = 0;
J(3,2) = ((sin(theta_2)*cos(theta_3)*cos(theta_4)-sin(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(sin(theta_2)*cos(theta_3)*sin(theta_4)+sin(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5+sin(theta_2)*cos(theta_3)*cos(theta_4)*a_4-sin(theta_2)*sin(theta_3)*sin(theta_4)*a_4+sin(theta_2)*cos(theta_3)*a_3;
J(3,3) = ((cos(theta_2)*sin(theta_3)*cos(theta_4)+cos(theta_2)*cos(theta_3)*sin(theta_4))*sin(theta_4+theta_3)+(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*sin(theta_4+theta_3))*a_5+cos(theta_2)*sin(theta_3)*cos(theta_4)*a_4+cos(theta_2)*cos(theta_3)*sin(theta_4)*a_4+cos(theta_2)*sin(theta_3)*a_3;
J(3,4) = ((cos(theta_2)*sin(theta_3)*cos(theta_4)+cos(theta_2)*cos(theta_3)*sin(theta_4))*sin(theta_4+theta_3)+(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*sin(theta_4+theta_3))*a_5+cos(theta_2)*cos(theta_3)*sin(theta_4)*a_4+cos(theta_2)*sin(theta_3)*cos(theta_4)*a_4;
J(3,5) = 0;

        %((sin(theta_2)*cos(theta_3)*cos(theta_4)-sin(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(sin(theta_2)*cos(theta_3)*sin(theta_4)+sin(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5+sin(theta_2)*cos(theta_3)*cos(theta_4)*a_4-sin(theta_2)*sin(theta_3)*sin(theta_4)*a_4+sin(theta_2)*cos(theta_3)*a_3
        %((cos(theta_2)*sin(theta_3)*cos(theta_4)+cos(theta_2)*cos(theta_3)*sin(theta_4))*sin(theta_4+theta_3)+(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*sin(theta_4+theta_3))*a_5+cos(theta_2)*sin(theta_3)*cos(theta_4)*a_4+cos(theta_2)*cos(theta_3)*sin(theta_4)*a_4+cos(theta_2)*sin(theta_3)*a_3
        %((cos(theta_2)*sin(theta_3)*cos(theta_4)+cos(theta_2)*cos(theta_3)*sin(theta_4))*sin(theta_4+theta_3)+(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*sin(theta_4+theta_3))*a_5+cos(theta_2)*cos(theta_3)*sin(theta_4)*a_4+cos(theta_2)*sin(theta_3)*cos(theta_4)*a_4
 

J(4,1) = 1;
J(4,2) = 0;
J(4,3) = 0;
J(4,4) = 0;
J(4,5) = 0;

J(5,1) = 0;
J(5,2) = -1;
J(5,3) = 0;
J(5,4) = 0;
J(5,5) = 1


end