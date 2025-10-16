function [center1, center2] = instantcenters(theta1, pos, L)
%INSTANTCENTERS Returns instant centers of velocity of a four-bar mechanism
%   [center1, center2] = instantcenters(theta1, pos, L) computes the
%   instant center of velocities for the four-bar mechanism at input angle
%   theta1 and geometric information in pos and L.

% there are 6 IC, four of which are the joint centers

[crank, coupler, follower, ground] = rodpositions(theta1(1), pos, L);
center1 = getcenter(crank, follower);
center2 = getcenter(coupler, ground);
end

function center = getcenter(link1, link2)
%GETCENTER Returns an instant center of velocity of a four-bar mechanism
%   center = getcenter(link1, link2) returns the point where the lines
%   defined by link1 and link2 intersect.

% A helpful reference is https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

p1 = link1(:, 1);
p2 = link1(:, 2);

p3 = link2(:, 1);
p4 = link2(:, 2);

a = det([p1'; p2']);
b = det([p3'; p4']);
c = det([p1(1), 1; p2(1), 1]);
d = det([p3(1), 1; p4(1), 1]);
e = det([p1(2), 1; p2(2), 1]);
f = det([p3(2), 1; p4(2), 1]);

den = det([c, e; d, f]);
px = det([a, c; b, d]) / den;
py = det([a, e; b, f]) / den;

if abs(den) < 1e-6
    center = [];
else
    center = [px; py];
end
end