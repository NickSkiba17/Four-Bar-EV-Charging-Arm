function h = drawfourbar(theta1, pos, L, h)
%DRAWFOURBAR Draws a four-bar linkage
%   h = drawfourbar(theta1, pos, L, []) plots a new instance of a four-bar
%   linkage given input angle theta1 and position and lengths of the four-
%   bar and end-effector links.
%
%   h = drawfourbar(theta1, pos, L, h) updates the four-bar's configuration
%   given a handle h to a previous call to the function
%
%   See alo DRAWCENTERS, ANIMATE, RODPOSITIONS, ENDEFFECTOR

[crank, coupler, follower, ground] = rodpositions(theta1(1), pos, L);
ee = endeffector(theta1(1), pos, L);
if nargin < 4 || isempty(h)
    % new drawing
    hcr = line(crank(1,:), crank(2,:), 'Color', 'red');
    hco = line(coupler(1,:), coupler(2,:), 'Color', 'green');
    hfo = line(follower(1,:), follower(2,:), 'Color', 'blue');
    hgr = line(ground(1,:), ground(2,:), 'Color', 'black');
    hee = line(ee(1,:), ee(2,:), 'LineStyle', '--', 'Color', 'cyan');
    heepts = line(ee(1,2), ee(2,2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'cyan', 'Color', 'cyan');
    h = [hcr, hco, hfo, hgr, hee, heepts];
else
    % update drawing
    set(h(1), 'XData', crank(1,:), 'YData', crank(2,:));
    set(h(2), 'XData', coupler(1,:), 'YData', coupler(2,:));
    set(h(3), 'XData', follower(1,:), 'YData', follower(2,:));
    set(h(4), 'XData', ground(1,:), 'YData', ground(2,:));
    set(h(5), 'XData', ee(1,:), 'YData', ee(2,:));
    set(h(6), ...
        'XData', [h(6).XData, ee(1,2)], 'YData', [h(6).YData, ee(2,2)]);
end
end