function mu = transmissionangle(theta1, ~, L)
%TRANSMISSIONANGLE Returns the transmission angle of a four-bar linkage
%   The transmission angle (relative angle between the coupler and
%   follower) is a metric for understanding how much torque is transmitted
%   from the input torque at the crank to the output torque at the joint
%   connecting the follower to ground without considering forces and
%   velocities.  The transmission angle should stay as close to 90 degrees
%   as possible for maximal transmission and stay away from mutlples of 180
%   degrees to avoid locking (no amount of input torque can move the output
%   link).
%
%   See also INPUTTORQUE

% A helpful resource: https://ocw.metu.edu.tr/pluginfile.php/6885/mod_resource/content/1/ch7/7-1.htm

d = L(1)^2 - L(2)^2 - L(3)^2 + L(4)^2;
mu = acos(-(d - 2 * L(1) * L(4) * cos(theta1(1))) / (2 * L(2) * L(3)));
end