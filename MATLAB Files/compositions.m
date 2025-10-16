function [rcom, vcom, acom] = compositions(r, beta, theta1, pos, L)
%COMPOSITION computer center-of-mass positions
%   [rcom, vcom, acom] = composition(r, beta, theta1, pos, L) returns the
%   location of the center of mass of each link, its velocity, and
%   acceleration.

[theta, thetadot, thetaddot] = jointangles(theta1, pos, L);
[crank, ~, follower, ~] = rodpositions(theta1, pos, L);

frames = [crank(:, 1:2), follower(:, 1)];
vframes = [crank(:, 3:4), follower(:, 3)];
aframes = [crank(:, 5:6), follower(:, 5)];

rcom = zeros(2, 3);
vcom = zeros(2, 3);
acom = zeros(2, 3);

for i = 1:3
    c = cos(theta(i) + beta(i));
    s = sin(theta(i) + beta(i));

    rcom(:, i) = frames(:, i) + r(i) * [c; s];
    vcom(:, i) = vframes(:, i) + r(i) * thetadot(i) * [-s; c];
    acom(:, i) = aframes(:, i) + r(i) ...
        * (thetaddot(i) * [-s; c] - thetadot(i)^2 * [c; s]);
end
end