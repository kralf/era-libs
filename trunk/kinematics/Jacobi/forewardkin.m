
syms  theta_1 theta_2 theta_3 theta_4 a_3 a_4 a_5 theta_5 theta_6



Rz_1 = rotz(theta_1);
Tz_1 = transz(a_3+a_4+a_5);
Rx_1 = rotx(pi/2);
A_1  = Rz_1 * Tz_1 * Rx_1;

Rz_2 = rotz(theta_2+pi/2);
Rx_2 = rotx(pi/2);
A_2  = Rz_2 * Rx_2;

Rz_3 = rotz(theta_3);
Tx_3 = transx(-a_3);
A_3  = Rz_3 * Tx_3;

Rz_4 = rotz(theta_4);
Tx_4 = transx(-a_4);
A_4  = Rz_4 * Tx_4;

Rz_5 = rotz(pi/2 - theta_4 - theta_3 - pi/2);
Rx_5 = rotx(pi/2);
A_5  = Rz_5 * Rx_5;

Rz_6 = rotz(theta_6);
Tz_6 = transz(a_5);
A_6  = Rz_6 * Tz_6;

H_1 = simple(A_1);
H_2 = simple(H_1*A_2);
H_3 = simple(H_2*A_3);
H_4 = simple(H_3*A_4);
H_5 = simple(H_4*A_5);
H_6 = simple(H_5*A_6);

%latex(H_1) 
%latex(H_2) 
%latex(H_3)
%latex(H_4) 
%latex(H_5) 
%latex(H_6)


%pos6 = simplify(H6(1:3,4));
%pos6 = simple(H5(1:3,3));
%simple(H6)