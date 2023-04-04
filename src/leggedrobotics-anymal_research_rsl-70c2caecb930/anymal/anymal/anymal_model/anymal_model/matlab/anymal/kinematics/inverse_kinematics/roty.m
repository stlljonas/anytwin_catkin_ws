function R = roty(beta)
%ROTZ Computes the elementary rotation matrix around the y axis

%   Author(s): C. Dario Bellicoso

R = [cos(beta)   0   sin(beta);
     0           1   0;
     -sin(beta)  0   cos(beta)];

end