function [ee, vee, aee] = endeffector(theta1, pos, L)
%ENDEFFECTOR Returns the motion details of the end effector
%   ee = endeffector(theta1, pos, L) returns a 2 x 2 matrix of positions
%   such that ee = [location of joint 2, location of end effector] in a
%   fixed world frame or 
%      [x2, ee_x;
%       y2, ee_y],
%   where (x2, y2) is the location of the second joint joining the
%   crank and coupler to rotate relative to each other and (ee_x, ee_y) is
%   the location of the end effector in the world frame.  The position of
%   the end effector relative to joint 2 is expressed using polar
%   coordinates (ree, delta) such that ree = L(5) and delta = pos(4).
%
%   [ee, vee, aee] = endeffector(theta1, pos, L) also returns the
%   end-effector velocity and acceleration in an inertial frame

ree = L(5);
delta = pos(4);

[theta, thetadot, thetaddot] = jointangles(theta1, pos, L);
[~, coupler, ~, ~] = rodpositions(theta1, pos, L);

c = cos(theta(2) + delta);
s = sin(theta(2) + delta);

ee = coupler(:,1) + ree * [c; s];
ee = [coupler(:,1), ee];

if numel(theta1) > 1
    vee = coupler(:,3) + ree * thetadot(2) * [-s; c];
    vee = [coupler(:,3), vee];
else
    vee = [];
end

if numel(theta1) > 2
    aee = coupler(:,5) ...
        + ree * (thetaddot(2) * [-s; c] - thetadot(2)^2 * [c; s]);
    aee = [coupler(:,5), aee];
else
    aee = [];
end
end