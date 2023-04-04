function R = rotz(alpha)
%ROTZ Computes the elementary rotation matrix around the z axis

%   Author(s): C. Dario Bellicoso
R = [cos(alpha)   -sin(alpha)   0;
     sin(alpha)    cos(alpha)   0;
     0             0            1];

end