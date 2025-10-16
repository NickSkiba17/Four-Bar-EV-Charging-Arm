function J = costfunction(x, x0, n)
%COSTFUNCTION returns the cost function for use with fmincon
%   costfunction(x, x0) computes a running cost and terminal cost.  The
%   goal is to keep the optimized motion as similar as possible to the
%   initial motion as defined by x0 (the running cost) and impacting the EV
%   charging port on a car at zero velocity (the terminal cost).
%
%   The running cost is equivalent to
%      sum(norm(motion(theta1, x) - motion(th1ref, x0))^2)
%   for theta1 between x(1) and x(2) and th1ref between x0(1) and x0(2).
%   The terminal cost is the end-effector velocity at th1end = x(2).
%
%   costfunction(x, x0, n) uses n points to compute the error between the
%   two motions
%
%   See also CONSTRAINTS, OPTIMIZE, FMINCON.

% terminal cost: end-effector velocity at theta1 max
th1max = x(2);
pos = x(3:6);
L = x(7:11);

pos0 = x0(3:6);

R = rot(-pos(3));
R0 = rot(-pos0(3));

params = sharedparameters(pos, L);
theta1dot = params.servovelocity;

if nargin < 3
    n = params.n;
end

[~, vee] = endeffector([th1max; theta1dot], pos, L);
e = vee(:, 2);
J = e' * e;

% running cost: error between current and initial path.
% The input angles of each motion is normalized to lie between an interval
% of [0, 1].  We then compare at each normalized point k.
for i = 0:n
    k = i / n;

    % actual position given decision variables
    ee = R * (eepos(k, x) - pos(1:2));

    % desired position given x0
    ee0 = R0 * (eepos(k, x0) - pos0(1:2));

    % add current error to running sum
    e = ee - ee0;
    J = J + e' * e;
end
end

function ee = eepos(k, x)
%EEPOS returns position of end effector given a normalized input in [0, 1]
th1min = x(1);
th1max = x(2);
pos = x(3:6);
L = x(7:11);

m = th1max - th1min;
th1 = m * k + th1min;
ee = endeffector(th1, pos, L);
ee = ee(:, 2);
end

function R = rot(gamma)
%ROT returns a rotation matrix that rotates vectors counter-clockwise
R = [cos(gamma), -sin(gamma); sin(gamma), cos(gamma)];
end