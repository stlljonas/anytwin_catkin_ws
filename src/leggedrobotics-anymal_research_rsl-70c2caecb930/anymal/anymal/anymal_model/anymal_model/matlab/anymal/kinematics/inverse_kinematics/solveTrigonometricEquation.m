function x = solveTrigonometricEquation(a,b,c,solFlag)
%SOLVETRIGONOMETRICEQUATION Solves a linear trigonometric equation.
%   [X] = SOLVETRIGONOMETRICEQUATION(A,B,C, SOLFLAG) solve following linear
%   trigonometric equation:
%           a*cos(x) + b*sin(x) = c
%
%   The solutoion can be found by solving:
%
%                          c
%   sin(x + alpha) = ---------------
%                     sqrt(a^2+b^2)
%
%   The parameter SOLFLAG (which can be either 1 or 2) allows the user to
%   choose which of the two solutions should be computed.

%   Author(s): C. Dario Bellicoso

delta = a*a + b*b - c*c;

if (abs(delta) < eps)
    delta = 0.0;
end

if (delta<0)
    disp(['delta<0!: ' num2str(delta)]);
    delta = 0;
end

if (solFlag == 1)
    x = atan2(a,b) - atan2(sqrt(delta), c);
elseif (solFlag == 2)
    x = atan2(a,b) + atan2(sqrt(delta), c);
end

end
