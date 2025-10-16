function animate(theta1bnds, pos, L, totaltime)
%ANIMATE Animate four-bar linkage
%   animate(theta1bnds, pos, L) shows the motion of a four-bar linkage with
%   input angles starting at theta1bnds(1) and ending at theta1bnds(2) with
%   the position and orientation of the four-bar defined by pos = [x1; y1;
%   gamma; delta] and lengths L = [L1; L2; L3; L4; ree].  The total time of
%   the animation is 5 seconds long.
%
%   animate(..., totaltime) run the animation for totaltime seconds.

% used default timing?
if nargin < 4
    totaltime = 5;
end

% use default range?
if isempty(theta1bnds)
    theta1bnds = inputrange(pos, L);
end

th1start = theta1bnds(1);
th1end = theta1bnds(2);

% get parameters
params = sharedparameters(pos, L);
framewidth = params.framewidth;
frameheight = params.frameheight;
evportx = params.evpos(1);
evporty = params.evpos(2);

% draw frame
rectangle('Position', [0, 0, framewidth, frameheight]);
line([evportx, evportx], [evporty, evporty], 'Marker', 'x');

% draw port
slit = params.slit;
line(slit(1, :), slit(2, :), 'Color', 'magenta');

% get max space taken up by four-bar + end-effector in a plot
bnds = minmaxposition(theta1bnds, pos, L);

d = 1/100;
d = [
    min(0, bnds(1)) - d, ...
    max(evportx, bnds(2)) + d, ...
    min(0, bnds(3)) - d, ...
    max(frameheight, bnds(4)) + d
    ];
axis(d);

% animate the motion by drawing the four-bar, frame, and end-effector
n = params.n;
theta1 = linspace(th1start, th1end, n);
theta1 = [theta1, theta1(end:-1:1)];

h = [];
hctrs = [];
for i = 1:2*n
    th1 = theta1(i);
    %% ANIMATE: Get the animate function working
    %   TODO: insert code that calls the two drawing functions drawfourbar
    %   and drawcenters using th1, pos, L, h, and hctrs.  Remember, we are
    %   updating the graphic handles h and hctrs with each iteration of the
    %   loop.
    % <------------INSERT YOUR CODE HERE------------>
    
    h = drawfourbar(th1, pos, L, h);
    hctrs = drawcenters(th1, pos, L, hctrs);
    
    pause(totaltime / (2 * n));

end
end