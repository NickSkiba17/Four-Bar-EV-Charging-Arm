function [eeinside, eeoutside] = minmaxee(th1bnds, pos, L)
%MINMAXEE Returns bounding boxes for the end effector inside and outside
%the frame
%   eeinside = minmaxee(theta1bnds, pos, L) returns a bounding box of the
%   motion of the end effector while inside the frame.  fmincon does not
%   like inf values, so if the motion is never inside the frame eeinside
%   represent the bounds for the the motion outside of the frame.  The
%   return vector has the format eeinside = [xmin, xmax, ymin, ymax, theta1
%   at xmin, theta1 at xmax, ...] based on a range of input joint angles
%   theta1bnds = [th1start, th1end] and other four-bar linkage information
%   in position pos and link lenghts L.
%
%   [eeinside, eeoutside] = minmaxee(...) also include eeoutside, which is
%   the bounding box of values for the portion of the end effector that
%   intersects the slit.  While it has the same format as eeinside, this
%   bounding box is most useful for the range of values along the vertical
%   of the endeffector with the frame at location of the slit.  An
%   intersection test determines what point between the end effector and
%   coupler are in contact with the frame.
%
%   See also MINMAXPOSITION, MINMAXINPUTTORQUE

n = 100;
th1min = th1bnds(1);
th1max = th1bnds(2);

params = sharedparameters(pos, L);
framewidth = params.framewidth;
slit = params.slit; 

eeinside = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
eeoutside = [inf, -inf, inf, -inf, NaN, NaN, NaN, NaN];
for theta1 = linspace(th1min, th1max, n)
    ee = endeffector(theta1, pos, L);
    if ee(1, 2) <= framewidth
        eeinside = minmax(theta1, ee, eeinside, 2);
    else
        eeint = intersection(ee, slit);
        eeoutside = minmax(theta1, eeint, eeoutside, 1);
    end
end

if any(isinf(eeinside(1:4)))
    for theta1 = linspace(th1min, th1max, n)
        ee = endeffector(theta1, pos, L);
        eeinside = minmax(theta1, ee, eeinside, 2);
    end
end

if any(isinf(eeoutside(1:4)))
    ee = endeffector(th1max, pos, L);
    eeint = intersection(ee, slit);
    eeoutside = minmax(th1max, eeint, eeoutside, 1);
end

end

function [p, t, u] = intersection(ee, slit)
% https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
p1 = ee(:, 1);
p2 = ee(:, 2);

p3 = slit(:, 1);
p4 = slit(:, 2);

x12 = p1(1) - p2(1);
x13 = p1(1) - p3(1);
x34 = p3(1) - p4(1);

y12 = p1(2) - p2(2);
y13 = p1(2) - p3(2);
y34 = p3(2) - p4(2);

a = det([x13, x34; y13, y34]);
b = det([x12, x13; y12, y13]);
c = det([x12, x34; y12, y34]);

t = a / c;
u = -b / c;

p = p3 + u * (p4 - p3);
end

function b = minmax(theta1, frame, b, k)
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

% https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect