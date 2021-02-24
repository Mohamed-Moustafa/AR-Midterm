function [q] = IK_Scara(p_global)

%constants
L1 = 0.4;
L2 = 0.5;
L3 = 0.5;
L4 = 0.1;


x = p_global(1);
y = p_global(2);
z = p_global(3);
phi = p_global(4);
%% Inverse kinematis
q3 = z - L1 + L4;

d = sqrt(x^2 + y^2);

q2 =  acos((d^2 - L2^2 - L3^2)/(2*L2*L3));

beta = atan2(y,x);

alpha = atan2( L3 * sin(q2), (L2+L3*cos(q2)));

q1 = -alpha+beta;

q4 = -(q1+q2) + phi;

q = [q1, q2, q3, q4];

end