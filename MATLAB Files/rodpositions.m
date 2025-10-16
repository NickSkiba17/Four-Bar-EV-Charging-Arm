function [crank, coupler, follower, ground] = rodpositions(theta1, pos, L)
%RODPOSITIONS Returns a four-bar's link positions at a given angle
%   crank = rodpositions(theta1, pos, L) returns a 2 x 2 matrix of
%   positions such that crank = [location of joint 1, location of joint 2]
%   in a fixed world frame or 
%      [x1, x2;
%       y1, y2],
%   where (x1 = pos(1), y1 = pos(2)) is the location of the joint joining
%   the crank and ground and (x2, y2) is the location of the joint joining
%   the coupler and follower.  The relative position of (x2, y2) is
%   computed in polar coordinates (L(1), theta1) and added to (x1, y1) for
%   the location of joint 2 in the world frame.
%
%   crank = rodpositions([theta1; theta1dot], pos, L) returns a 2 x 4
%   matrix of positions and velocities of the crank in a fixed world frame
%   such that
%      [x1, x2, vx1, vx2;
%       y1, y2, vy1, vy2].
%
%   crank = rodpositions([theta1; theta1dot; thetaddot], ...) returns a 2 x
%   6 matrix of positions, velocities, and accelerations such that
%      [x1, x2, vx1, vx2, ax1, ax2;
%       y1, y2, vy1, vy2, ay1, ay2].
%
%   [crank, coupler, follower, ground] = rodpositions(...) also returns the
%   other links and their joint centers: 
%      crank = [location of joint 1, location of joint 2]
%      coupler = [location of joint 2, location of joint 3]
%      follower = [location of joint 4, location of joint 3]
%      ground = [location of joint 1, location of joint 4]

[theta, thetadot, thetaddot] = jointangles(theta1, pos, L);

c = zeros(1, 4);
s = zeros(1, 4);
for i = 1:4
    c(i) = cos(theta(i)); 
    s(i) = sin(theta(i));
end

frame1 = pos(1:2);
frame2 = frame1 + L(1) * [c(1); s(1)];
frame3 = frame2 + L(2) * [c(2); s(2)];

frame5 = frame1 + L(4) * [c(4); s(4)];
frame4 = frame5 + L(3) * [c(3); s(3)];
% frame4 must always equal frame3 b/c of a loop closure constraint
% this is also true for their derivatives

crank = [frame1, frame2];
coupler = [frame2, frame3];
follower = [frame5, frame4];
ground = [frame1, frame5];

if numel(theta1) > 1
    vframe = zeros(2, 5);

    % there are dependencies when computing velocities, which is reflected
    % with the parent array and the list to iterate over
    parent = [0 1 2 5 1];
    for i = [2, 3, 5, 4]
        j = parent(i);
        vframe(:, i) = vframe(:, j) + L(i - 1) * thetadot(i - 1) ...
            * [-s(i - 1); c(i - 1)];
    end

    crank = [crank, vframe(:, 1:2)];
    coupler = [coupler, vframe(:, 2:3)];
    follower = [follower, vframe(:, [5, 4])];
    ground = [ground, vframe(:, [1, 5])];
end

if numel(theta1) > 2
    aframe = zeros(2, 5);

    % respect dependencies when computing accelerations
    for i = [2, 3, 5, 4]
        j = parent(i);
        aframe(:, i) = aframe(:, j) ...
            + L(i - 1) * thetaddot(i - 1) * [-s(i - 1); c(i - 1)] ...
            - L(i - 1) * thetadot(i - 1)^2 * [c(i - 1); s(i - 1)];
    end

    crank = [crank, aframe(:, 1:2)];
    coupler = [coupler, aframe(:, 2:3)];
    follower = [follower, aframe(:, [5, 4])];
    ground = [ground, aframe(:, [1, 5])];
end
end