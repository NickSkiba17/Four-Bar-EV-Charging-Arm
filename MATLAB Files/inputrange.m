function [theta1bnds, type] = inputrange(pos, L)
%INPUTRANGE Returns valid angle range of the four-bar linkage
%   inputrange(pos, L) returns valid start and end values for the input
%   crank angle given pos(3) = gamma, the angle of the ground link relative
%   to the horizontal, and link lengths L(1:4) = [L1; L2; L3; L4].
%
%   [theta1bnds, type] = inputrange(pos, L) also returns the type of bound


% A helpful reference is https://mechanicaldesign101.com/wp-content/uploads/2017/12/T1-Four-bar-Linkage-Analysis-revised.pdf

cmin = -((L(2) - L(3))^2 - L(1)^2 - L(4)^2) / (2 * L(1) * L(4));
cmax = -((L(2) + L(3))^2 - L(1)^2 - L(4)^2) / (2 * L(1) * L(4));

if abs(cmin) > 1 && abs(cmax) > 1
    type = 'none';
    th1start = 0;
    th1end = 2 * pi;
elseif abs(cmax) > 1
    type = 'min';
    th1start = acos(cmin);
    th1end = 2*pi - th1start;
elseif abs(cmin) > 1
    type = 'max';
    th1start = -acos(cmax);
    th1end = -th1start;
else
    type = 'both';
    th1start = acos(cmin);
    th1end = acos(cmax);
end

theta1bnds = pos(3) + [th1start, th1end];
end