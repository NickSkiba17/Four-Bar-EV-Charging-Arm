function h = drawcenters(theta1, pos, L, h)
%DRAWCENTERS Draws a four-bar linkage
%   h = drawcenters(theta1, pos, L, []) plots a new instance of a instant
%   center of velocities given input angle theta1 and position and lengths
%   of the four-bar and end-effector links.
%
%   h = drawcenters(theta1, pos, L, h) updates the centers's configuration
%   given a handle h to a previous call to the function
%
%   See alo DRAWFOURBAR, INSTANTCENTERS, ANIMATE

[ctr1, ctr2] = instantcenters(theta1(1), pos, L);

if nargin < 4 || isempty(h)
    % new drawing
    hctr1 = drawcenter(ctr1, []);
    hctr2 = drawcenter(ctr2, []);

    h = [hctr1, hctr2];
else
    % update drawing
    drawcenter(ctr1, h(1));
    drawcenter(ctr2, h(2));
end
end

function h = drawcenter(ctr, h)
if isempty(h) && isempty(ctr)
    h = line('XData', [], 'Marker', 'o');
elseif isempty(h)
    h = line(ctr([1, 1]), ctr([2, 2]), 'Marker', 'o');
elseif isempty(ctr)
    set(h, 'XData', [], 'YData', []);
else
    set(h, 'XData', ctr([1, 1]), 'YData', ctr([2, 2]));
end
end