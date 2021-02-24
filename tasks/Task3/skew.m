function [T] = skew(vector)

T= zeros(3,3);
T(1,2)= -vector(3);
T(1,3)= vector(2);
T(2,1)= vector(3);
T(2,3)=-vector(1);
T(3,1)=-vector(2);
T(3,2)=vector(1);

end

