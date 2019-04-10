clc

clear

syms q1 q2 Q2 Q3 u1 l1 l2 dl1 dl2 xx real

e1= 0.154; a1= 0.550; e2= 0.130; a2= 0.600;


gam1 = atan(e1/a1); gam2 = atan(e2/a2); b1 = sqrt(a1*a1 + e1*e1); b2 = sqrt(a2*a2 + e2*e2);
gam3 = acos((l1*l1 - b1*b1 - b2*b2)/(-2.0*b1*b2)); q2 = gam1 + gam2 + gam3 - pi/2;

e1= 0.160; a1= 0.750; e2= 0.0765; a2= 0.167;

gam1 = atan(e1/a1); gam2 = atan(e2/a2); b1 = sqrt(a1*a1 + e1*e1); b2 = sqrt(a2*a2 + e2*e2);
gam3 = acos((l2*l2 - b1*b1 - b2*b2)/(-2.0*b1*b2)); q3 = gam1 + gam2 + gam3 - pi;


%Derivatives
u2 = vpa(simplify(diff(q2,l1)*dl1),5); u3 = vpa(simplify(diff(q3,l2)*dl2),5);

%Relative twists
w_1_01 = [0 0 1]'; w_2_12 = [0 0 1]'; w_3_23 = [0 0 1]'; w_4_34 = [0 0 0]';

% X axis in the crane frame, Z axis UP. Q2 is from horizontal line to x_2,
% Q3 is from x2 to x3
R01 = Rxd(180)*Rzd(90)'*Rz(q1); R12 = Rxd(90)*Rz(q2); R23 = Rz(q3); R34 = Rzd(-39.4);

%Q2=alpha(q2), Q3=alpha(q3);
p_1_12 = [0 0 xx]'; p_2_23 = [1.5 0 0]'; p_3_34 = [0.2055 0 0]'; p_4_4e = [0.992 0 0]';
p_1_2e = R12*(p_2_23 + R23*(p_3_34 + R34*p_4_4e)); p_1_3e = R12*R23*(p_3_34 + R34*p_4_4e);

w_1_12 = R12*w_2_12; w_1_23 = R12*R23*w_3_23;

J_ = [sks(w_1_12)*p_1_2e, sks(w_1_23)*p_1_3e]; UL = [diff(q2,l1), 0; 0, diff(q3,l2)];
J = J_*UL; J1 = R01*J;



%%%%% Check with the new Jacobian as in the papaer
p_0_1e = R01*( p_1_12 + R12*(p_2_23 + R23*(p_3_34 + R34*p_4_4e)));
p_0_2e = R01*R12*(p_2_23 + R23*(p_3_34 + R34*p_4_4e));
p_0_3e = R01*R12*R23*(p_3_34 + R34*p_4_4e);


w_0_01 =  R01*w_1_01; w_0_12 =  R01*R12*w_2_12; w_0_23 =  R01*R12*R23*w_3_23;

J2_ = [sks(w_0_01)*p_0_1e, sks(w_0_12)*p_0_2e, sks(w_0_23)*p_0_3e];
UL2 = [1,0,0; 0, diff(q2,l1), 0; 0, 0, diff(q3,l2)];
J2 = J2_*UL2;