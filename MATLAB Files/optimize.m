function [xopt, fval, exitflag, output] = optimize(x0)
%OPTIMIZE Returns optimized parameters of a four-bar linkage
%   xopt = optimize(x0) returns optimized parameters xopt given a reference
%   model x0.  The input and output vectors consists of
%        x(1): start angle of the crank
%        x(2): end angle of the crank
%        x(3): x position of input joint at the crank relative to the frame
%        x(4): y position of input joint at the crank relative to the frame
%        x(5): fixed angle of the ground link relative to the horizontal
%        x(6): fixed angle of the end-effector relative to the coupler
%        x(7): length of crank
%        x(8): length of coupler
%        x(9): length of follower
%        x(10): length of ground link
%        x(11): location of end effector (ee)
%   These values are also referred to as
%        x = [th1start; th1end; x1; y1; gamma; delta; L1; L2; L3; L4; ree]
%
%   [xopt, fval, exitflag, output] = optimize(x0) returns fmincon results
%
%   See also CONSTRAINTS, COSTFUNCTION, FMINCON.

% x = [th1start; th1end; x1; y1; gamma; delta; L1; L2; L3; L4; ree];

% cost function
J = @(x) costfunction(x, x0);
c = @(x) constraints(x);

% unpack decision variables
% th1start = x0(1);
% th1end = x0(2);
pos = x0(3:6);
L = x0(7:11);

% get frame width and height
params = sharedparameters(pos, L);
framewidth = params.framewidth;
frameheight = params.frameheight;

Lmax = max(framewidth, frameheight);


%%%%%% If necessary, set upper and lower bounds to range of values you
%%%%%% think you can cut and assemble in your design

% set lower bounds on the decision variables
lb = [
    -inf; % th1start, radians
    -inf; % th1end, radians
    0; % x1, m
    0; % y1, m
    -inf; % gamma, radians
    -inf; % delta, radians
    0.1 * Lmax; % L1, m
    0.1 * Lmax; % L2, m
    0.1 * Lmax; % L3, m
    0.1 * Lmax; % L4, m
    0; % ree, m
    ];

% set upper bounds on the decision variables
ub = [
    inf; % th1start
    inf;  % th1end
    framewidth; % x1
    frameheight; % y1
    inf; % gamma
    inf; % delta
    Lmax; % L1
    Lmax; % L2
    Lmax; % L3
    Lmax; % L4
    0.5 * Lmax; % ree
    ];

% use an sqp solver (testing often converged with this solver and a
% reasonable x0)
o = optimoptions('fmincon', 'Algorithm', 'sqp', ...
    'Display', 'iter-detailed', 'MaxFunctionEvaluations', inf);

[xopt, fval, exitflag, output] = fmincon( ...
    J, x0, [], [], [], [], lb, ub, c, o);
end