function [Jt,J1,J2,J3,J4] = Jt_Calibration(q,theta)
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

L1 = 0.4;
L2 = 0.5;
L3 = 0.5;
L4 = 0.1;

 
T = FK_Scara(q,theta);
T(1:3,4) = [0;0;0];
 

Td= Rz(q1)*Rzd(theta(1))...
    * Tz(L1) * Tx(L2) * Rz(q2)*Rz(theta(2))...
    * Tx(L3)* Rz(q4)* Rz(theta(4))...
    * Tz(-L4) * Tz(q3)*Tz(theta(3)) /T ;
        
J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

Td= Rz(q1)*Rz(theta(1))...
    * Tz(L1) * Tx(L2) * Rz(q2)*Rzd(theta(2))...
    * Tx(L3)* Rz(q4)* Rz(theta(4))...
    * Tz(-L4) * Tz(q3)*Tz(theta(3)) /T ;
        
J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

Td= Rz(q1)*Rz(theta(1))...
    * Tz(L1) * Tx(L2) * Rz(q2)*Rz(theta(2))...
    * Tx(L3)* Rz(q4)* Rz(theta(4))...
    * Tz(-L4) * Tz(q3)*Tzd(theta(3)) /T ;
        
J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

Td= Rz(q1)*Rz(theta(1))...
    * Tz(L1) * Tx(L2) * Rz(q2)*Rz(theta(2))...
    * Tx(L3)* Rz(q4)* Rzd(theta(4))...
    * Tz(-L4) * Tz(q3)*Tz(theta(3)) /T ;
        
J4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

  
Jt = [J1 J2 J3 J4];
end

