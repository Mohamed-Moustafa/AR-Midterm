function [deltaT, Kc] = Compute_Deflection_Scara(p_global, Force)
%COMPUTE_DEFLECTION_RPP Summary of this function goes here

%% Constant stuff
d = [0.2 0.05 0.05 0.01]; % Cylinder diameter
k0 = [4 1 1 0.5]*10^6; % actuator stiffness
E = 7.0000e+10; % Young's modulus
G = 2.5500e+10; % shear modulus
Link_lengths = [0.4 0.5 0.5 0.1]; %Link length


%for cylinder
S = pi*d.^2/4;
Iy = pi*d.^4/64;
Iz = pi*d.^4/64;

% Let's get the stiffness matrixes of the robot  
[k11_1, k12_1, k21_1, k22_1] = k_cylinder(E, G, d(1), Link_lengths(1), S(1), Iy(1), Iz(1));
[k11_2, k12_2, k21_2, k22_2] = k_cylinder(E, G, d(2), Link_lengths(2), S(2), Iy(2), Iz(2));
[k11_3, k12_3, k21_3, k22_3] = k_cylinder(E, G, d(3), Link_lengths(3), S(3), Iy(3), Iz(3));
[k11_4, k12_4, k21_4, k22_4] = k_cylinder(E, G, d(4), Link_lengths(4), S(4), Iy(4), Iz(4));


%% Extract the global position

x = p_global(1);
y = p_global(2);
z = p_global(3);

theta = zeros(1,4);
%% Get the inverse kinematics and forward kinematics

q = IK_Scara(p_global);

[T, R1, R2,R3,R4] = FK_Scara(q, theta);

%% Transforming the q from local frame to the global frame

Q1 = [R1, zeros(3,3);
     zeros(3,3), R1];

Q2 = [R2, zeros(3,3);
     zeros(3,3), R2];

Q3 = [R3, zeros(3,3);
     zeros(3,3), R3];

Q4 = [R4, zeros(3,3);
     zeros(3,3), R4];

 
 %% Global stiffness matrices for the 1st link 

K11_Link1 = Q1*k11_1*Q1';
K12_Link1 = Q1*k12_1*Q1';
K21_Link1 = Q1*k21_1'*Q1';
K22_Link1 = Q1*k22_1*Q1';

%% Global stiffness matrices for the 2nd link 

K11_Link2 = Q2*k11_2*Q2';
K12_Link2 = Q2*k12_2*Q2';
K21_Link2 = Q2*k21_2'*Q2';
K22_Link2 = Q2*k22_2*Q2';


%% Global stiffness matrices for the 3rd link 

K11_Link3 = Q3*k11_3*Q3';
K12_Link3 = Q3*k12_3*Q3';
K21_Link3 = Q2*k21_3'*Q3';
K22_Link3 = Q3*k22_3*Q3';

%% Global stiffness matrices for the 2nd link 

K11_Link4 = Q4*k11_4*Q4';
K12_Link4 = Q4*k12_4*Q4';
K21_Link4 = Q4*k21_4'*Q4';
K22_Link4 = Q4*k22_4*Q4';


%% Calculate Lamdas
[lambdaR1, lambdaE1] = lambda(6);
[lambdaR2, lambdaE2] = lambda(6);
[lambdaR3, lambdaE3] = lambda(3);
[lambdaR4, lambdaE4] = lambda(6);


Full_Matrix = [ Eq_maker('rigid_base', 1, 8);
    Eq_maker('flexible_link', 1, 8, K11_Link1, K12_Link1, K21_Link1, K22_Link1);
    Eq_maker('elastic_joint', 2, 8, lambdaR1, lambdaE1, k0(1));
    Eq_maker('flexible_link', 3, 8, K11_Link2, K12_Link2, K21_Link2, K22_Link2);
    Eq_maker('elastic_joint', 4, 8, lambdaR2, lambdaE2, k0(2));
    Eq_maker('flexible_link', 5, 8, K11_Link3, K12_Link3, K21_Link3, K22_Link3);
    Eq_maker('elastic_joint', 6, 8, lambdaR3, lambdaE3, k0(3));
    Eq_maker('flexible_link', 7, 8, K11_Link4, K12_Link4, K21_Link4, K22_Link4);
    Eq_maker('elastic_joint', 8, 8, lambdaR4, lambdaE4, k0(4));
    Eq_maker('external_force', 8, 8)];

A = Full_Matrix(1:102, 1:102);

B = Full_Matrix(1:102, 103:108);

C = Full_Matrix(103:108, 1:102);

D = Full_Matrix(103:108, 103:108);
Kc =  D - C*(A\B);
%Kc =  D - C/A*B;
%% deflection computation
deltaT = Kc \ Force; 

end

