%% Constants

% Stiffness coefficients
k_const = [4 1 1 0.5]*10^6;

% Stiffness matrix
K_t = diag(k_const);

% Number of experiments
N = 30;

% initialize thetas
theta = zeros(1,4);

% Initialize the A matrices
At_1 = zeros(4,4);
At_2 = zeros(4,1);
%% Getting the True stiffness matrix
for n = 1:N
    % Random Wrench vector
    w = randn(6,1)*1000/4;

    % Random angle
    q1 = randn(1,1)*pi/3;
    q2 = randn(1,1)*pi/3;
    q3 = rand(1,1);
    q4 = randn(1,1)*pi/3;
    
    q = [q1 q2 q3 q4];
   
    
    Jt = Jt_Scara(q, theta);
    Jt11= Jt(:,1);
    Jt22= Jt(:,2);
    Jt33= Jt(:,3);
    Jt44= Jt(:,4);
    
    
    Jt1= [Jt11(1:3) ;Jt11(6)];
    Jt2= [Jt22(1:3) ;Jt22(6)];
    Jt3= [Jt33(1:3) ;Jt33(6)];
    Jt4= [Jt44(1:3) ;Jt44(6)];
    % random noise
    eps = randn(6,1)*1e-05;
   
    % Calculate the deflection vector
    dt = (Jt / K_t * Jt')*w + eps;
    
    wt = [w(1:3);w(6)];
    
    % Calculate the A matrices
    A1 = Jt1 * Jt1' * wt;
    A2 = Jt2 * Jt2' * wt;
    A3 = Jt3 * Jt3' * wt;
    A4 = Jt4 * Jt4' * wt;
    
    A = [A1, A2, A3,A4];
    
    delta_t =[dt(1:3);dt(6)];
    
    At_1 = At_1 + A'*A;
    At_2 = At_2 + A'*delta_t;
end

%% Calculate the real compliance vector
Kc = At_1 \ At_2;

Ks = 1./Kc
