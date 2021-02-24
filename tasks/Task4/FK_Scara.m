function [FK, R1, R2 , R3, R4] = FK_Scara(q,theta)

%% Constants
L1 = 0.4;
L2 = 0.5;
L3 = 0.5;
L4 = 0.1;



%% Getting the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

T1 =Tz(L1);

T2 = T1 *  Rz(q1)*Rz(theta(1)) ;

T3 = T2 * Tx(L2) * Rz(q2)*Rz(theta(2));

FK = T3 * Tx(L3)* Rz(q4)* Rz(theta(4)) * Tz(-L4) * Tz(q3)*Tz(theta(3));

R1 = T2(1:3,1:3);

R2 = T3(1:3,1:3);

R3 = T3(1:3,1:3);

R4 = FK(1:3,1:3);
end

