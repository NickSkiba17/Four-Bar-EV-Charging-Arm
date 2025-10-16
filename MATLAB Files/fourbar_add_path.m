function fourbar_add_path()
% fourbar_add_path: Adds required folders to Matlab's path
%   A standard Matlab library setup is to have an xyz_add_path function
%   that ensures all library files can be accessed after being called once.
%   You must call this function at least once after starting Matlab, but
%   before you can start coding.

% The code is modified from https://github.com/fevrem/TROPIC/blob/develop/TROPIC_add_path.m

% make sure it's running from root
cur = fullfile(pwd);

% add relevant directories to the path
addpath(fullfile(cur, 'utils'));
addpath(fullfile(cur, 'kinematics'));
addpath(fullfile(cur, 'dynamics'));
addpath(fullfile(cur, 'optimization'));
end