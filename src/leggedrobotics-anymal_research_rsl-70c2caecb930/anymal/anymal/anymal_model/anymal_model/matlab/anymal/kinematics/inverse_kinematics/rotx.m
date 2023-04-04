function R = rotx(gamma)
%ROTZ Computes the elementary rotation matrix around the x axis

%   Author(s): C. Dario Bellicoso

R = [1   0           0;
     0   cos(gamma) -sin(gamma);
     0   sin(gamma)  cos(gamma)];

end