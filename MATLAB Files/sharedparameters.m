function params = sharedparameters(pos, L)
%SHAREDPARAMETERS Returns a struct of shared parameters
%   sharedparameters(pos, L) returns a struct of shared parameters that
%   would otherwise have to be passed across several functions.

%%%%%%%%%%%%%%%%%%%%% UPDATE THESE VALUES TO REFLECT YOUR DESIGN
width = 0.42; % width of frame in meters
height = 0.48; % height of frame in meters

rho = 750; % density of MDF (kg / m^3)

% inertial properties of crank, coupler and follower these are each 3 x 1
% vectors, e.g.,
%    m = [mass of crank; mass of coupler; mass of follower];
% and similarly for the other quantities

% mass of each link (kg)
m = rho * (L(1:3)) .* (L(1:3) / 2) * (1/4 * 0.254); 
% moment of inertia of each link (kg m^2)
Icom = 1/12 * m .* (4 * L(1:3).^2 + (L(1:3) / 2).^2);
% distance of center-of-mass from a joint center in (m)
rcom = L(1:3) / 2;
% angle of a center-of-mass position relative to its corresponding link (radians)
%
% E.g., the center of mass of the crank (a.k.a. link 1) is located at
%    pos(1:2) + rcom(1) * [cos(theta1 + beta(1)); sin(theta1 + beta(1))],
% where pos(1:2) = [x1; y1] is the position of joint 1 in a world frame.
beta = [0; 0; 0];
%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%  %%%%%%%%%%%%%%%%%%%%%  %%%%%%

% there are two solutions to a four-bar linkage that can achieve the same
% configuration (known as the elbow-up vs. elbow-down configuration); while
% rarely expected to be used set to true to see the other possible motion
params.flip = false;

% several functions compute the entire motion of the four-bar mechanism and
% collect the data.  Here we set a common number of points to generate.
params.n = 100;

% servo properties taken from (1)-(2); (3) has useful info for computing
% torque:
% (1) https://hitecrcd.com/hs-425bb-deluxe-ball-bearing-standard-servo/
% (2) https://www.servocity.com/hs-425bb-servo/
% (3) https://en.wikipedia.org/wiki/Servo_(radio_control)
params.servotorque = .402;% <------------INSERT YOUR CODE HERE------------>; % Nm
params.servovelocity = 6.545;% <------------INSERT YOUR CODE HERE------------>; % radians / s
params.servorange = deg2rad(180);% <------------INSERT YOUR CODE HERE------------>; % radians

% assumed center of mass parameters of the links; these values should be
% modified with reasonable values which will
%   (1) help establish a weight limit for each link
%   (2) help fmincon converge to a feasible solution
params.m = m; % kg
params.Icom = Icom; % kg m^2
params.rcom = rcom; % m
params.beta = beta; % radians

params.transmissionangle = [deg2rad(90-50), deg2rad(90+50)]; % radians

% update to your actual frame width and height
params.framewidth = width;
params.frameheight = height;

d = 2 * 0.0254; % half of slit height
params.slit = [width, width; 0.5 * height + [-d, d]];

d = 6 * 0.0254; % EV car port offset
params.evpos = [width + d; 0.5 * height];
end