function [Kc] = Kc_Scara_VJM(Jt)

k=[4 1 1 0.5]*10^6;
% K theta matrix
Kt = diag(k);

% Analytical solution
Kc=pinv(Jt/Kt*Jt');
end