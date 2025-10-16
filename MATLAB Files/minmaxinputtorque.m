function [btau, bmu] = minmaxinputtorque(theta1bnds, pos, L, params)
%MINMAXINPUTTORQUE Returns the min and max torque along a motion
%   btau = minmaxinputtorque(theta1bnds, pos, L, params) computes the min
%   and max torque values for a motion between th1start = theta1bnds(1) and
%   th1end = theta1bnds(2) and other geometric information contained in in
%   pos and L.  The format of btau is
%      btau = [torque min, torque max, ...
%         theta1 value of min torque, theta 1 value of max torque]
%
%   [btau, bmu] = minmaxinputtorque(theta1bnds, pos, L, params) also return
%   bounds on the transmission angle

if nargin < 4 || isempty(params)
    params = sharedparameters(pos, L);
end

n = 100;
lb = theta1bnds(1);
ub = theta1bnds(2);
btau = [inf, -inf, NaN, NaN];
bmu = [inf, -inf, NaN, NaN];
for theta1 = linspace(lb, ub, n)
    tau = inputtorque(theta1, pos, L, params);
    btau = minmax(theta1, tau, btau);

    mu = transmissionangle(theta1, pos, L);
    bmu = minmax(theta1, mu, bmu);
end
end

function b = minmax(theta1, qty, b)
%MINMAX Returns updated min/max values and the angles at which they occur

% min value
a = [b(3), theta1];
[v, j] = min([b(1), qty]);
b(1) = v;
b(3) = a(j);

% max value
a(1) = b(4);
[v, j] = max([b(2), qty]);
b(2) = v;
b(4) = a(j);
end