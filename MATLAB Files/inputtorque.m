function [tau, f] = inputtorque(theta1, pos, L, params)
%INPUTTORQUE Returns the input torque and joint forces of the four-bar
%   tau = inputtorque(theta1, pos, L, params) computes the input torque at
%   joint 1 of the crank for the desired configuration of the four bar at
%   input angle theta1.
%
%   [tau, f] = inputtorque(theta1, pos, L, params) also returns the forces
%
%   See also TRANSMISSIONANGLE

if nargin < 4 || isempty(params)
    params = sharedparameters(pos, L);
end

m = params.m;
Icom = params.Icom;
rcom = params.rcom;
beta = params.beta;

if numel(theta1) < 2
    theta1 = [theta1; params.servovelocity; 0];
elseif numel(theta1) < 3
    theta1 = [theta1; 0];
end

[~, ~, thetaddot] = jointangles(theta1, pos, L);
[crank, coupler, follower, ~] = rodpositions(theta1, pos, L);
rods = [crank(:, 1), coupler(:, 1:2), follower(:, 1)];
[rcoms, ~, acoms] = compositions(rcom, beta, theta1, pos, L);

% place joints in com coordinates
% joints 1 and 2 in com 1 coords => indices (:, 1:2)
% joints 2 and 3 in com 2 coords => (:, 3:4)
% joints 3 and 4 in com 3 coords => (:, 5:6)
joints = [
    rods(:, 1) - rcoms(:, 1), rods(:, 2) - rcoms(:, 1), ...
    rods(:, 2) - rcoms(:, 2), rods(:, 3) - rcoms(:, 2), ...
    rods(:, 3) - rcoms(:, 3), rods(:, 4) - rcoms(:, 3)
    ];

A = [
    % tau, f1x, f1y, f2x, f2y, f3x, f3y, f4x, f4y
    0, 1, 0, -1, 0, 0, 0, 0, 0;
    0, 0, 1, 0, -1, 0, 0, 0, 0;
    1, -joints(2,1), joints(1,1), joints(2,2), -joints(1,2), 0, 0, 0, 0;

    0, 0, 0, 1, 0, -1, 0, 0, 0;
    0, 0, 0, 0, 1, 0, -1, 0, 0;
    0, 0, 0, -joints(2,3), joints(1,3), joints(2,4), -joints(1,4), 0, 0;

    0, 0, 0, 0, 0, 1, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 1, 0, 1;
    0, 0, 0, 0, 0, -joints(2,5), joints(1,5), -joints(2,6), joints(1,6)
    ];

g = [0; -9.81];

b = [
    -m(1) * g + m(1) * acoms(:,1);
    Icom(1) * thetaddot(1);

    -m(2) * g + m(2) * acoms(:,2);
    Icom(2) * thetaddot(2);

    -m(3) * g + m(3) * acoms(:,3);
    Icom(3) * thetaddot(3)
    ];

warning('off');
x = A \ b;
warning('on');

tau = x(1);
f = reshape(x(2:9), [2, 4]);
end