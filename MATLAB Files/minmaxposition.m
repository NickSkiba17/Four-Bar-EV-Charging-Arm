function [bnds, rodbnds, bcr, bco, bfo, bgr, bee] = minmaxposition(theta1bnds, pos, L)
%MINMAXPOSITIONS Returns bounding boxes for each link and the end effector
%   bnds = minmaxposition(theta1bnds, pos, L) returns a bounding box of the
%   motion of the four-bar links and the end effector with bnds = [xmin,
%   xmax, ymin, ymax, theta1 at xmin, theta1 at xmax, ...] based on a range
%   of input joint angles theta1bnds = [th1start, th1end] and other
%   four-bar linkage information in position pos and link lenghts 
%   L.
%
%   [bnds, rodbnds] = minmaxposition(theta1bnds, pos, L) also return a
%   bounding box of the motion of only the four-bar links (i.e., excluding
%   the end effector).
%
%   [bnds, rodbnds, bcr, bco, ...] = minmaxposition(theta1bnds, pos, L)
%   also return a bounding box of the motion of just a particular link,
%   like the crank (bcr), coupler (bco), follower (bfo), ground (bgr), and
%   end effector (bee) in that order.
%
%   See also MINMAXEE, MINMAXINPUTTORQUE

n = 100;

lb = theta1bnds(1);
ub = theta1bnds(2);

bee = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
bcr = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
bco = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
bfo = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
bgr = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];

% loop through the entire motion and log min/max coordinate values
for theta1 = linspace(lb, ub, n)
    ee = endeffector(theta1, pos, L);
    bee = minmax(theta1, ee, bee, 2);

    [crank, coupler, follower, ground] = rodpositions(theta1, pos, L);

    bcr = minmax(theta1, crank, bcr, 2);
    bco = minmax(theta1, coupler, bco, 2);
    bfo = minmax(theta1, follower, bfo, 1);
    bgr = minmax(theta1, ground, bgr, 1);
end

% merge the individual bounding boxes into larger boxes
bnds = zeros(1, 8);
rodbnds = zeros(1, 8);
for i = 1:4
    v = [bcr(i), bco(i), bfo(i), bgr(i), bee(i)];
    a = [bcr(i + 4), bco(i + 4), bfo(i + 4), bgr(i + 4), bee(i + 4)];
    if mod(i, 2) == 1
        [v, j] = min(v);
    else
        [v, j] = max(v);
    end
    bnds(i) = v;
    bnds(i + 4) = a(j);

    v = [bcr(i), bco(i), bfo(i), bgr(i)];
    a = [bcr(i + 4), bco(i + 4), bfo(i + 4), bgr(i + 4)];
    if mod(i, 2) == 1
        [v, j] = min(v);
    else
        [v, j] = max(v);
    end
    rodbnds(i) = v;
    rodbnds(i + 4) = a(j);
end
end

function b = minmax(theta1, frame, b, k)
%MINMAX Returns updated min/max values and the angles at which they occur

x = frame(1, k);
y = frame(2, k);
cmp = [x, x, y, y];

for i = 1:4
    a = [b(i + 4), theta1];
    if mod(i, 2) == 1
        [v, j] = min([b(i), cmp(i)]);
    else
        [v, j] = max([b(i), cmp(i)]);
    end
    b(i) = v;
    b(i + 4) = a(j);
end
end