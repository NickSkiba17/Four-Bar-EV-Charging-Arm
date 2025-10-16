function [theta, thetadot, thetaddot] = jointangles(theta1, pos, L)
%JOINTANGLES Returns the joint angles of a four-bar mechanism
%   theta = jointangles(theta1, pos, L) returns joint angles based on the
%   input angle theta1, orientation gamma = pos(3), and link lengths
%   L(1:4).
%
%   [theta, thetadot, thetaddot] = jointangles(theta1, pos, L) also returns
%   angular velocity and acceleration of the joints
%
%   See also SOLVEANGLE

params = sharedparameters(pos, L);
flip = params.flip;

gamma = pos(3);
th1 = theta1(1);
s1 = sin(th1);
c1 = cos(th1);
c14 = cos(th1 - gamma);

a = L(1) * s1 - L(4) * sin(gamma);
b = L(1) * c1 - L(4) * cos(gamma);
% for theta2 => (a, b), theta3 => (-a, -b)

z = L(1)^2 + L(4)^2 - 2 * L(1) * L(4) * c14;
z2 = (L(2)^2 - L(3)^2 + z) / (2 * L(2));
z3 = (L(3)^2 - L(2)^2 + z) / (2 * L(3));

if ~flip
    % take +'ve solution for theta2 and -'ve for theta3
    th2 = solveangle(a, b, z2);
    [~, th3] = solveangle(-a, -b, z3);
else
    % flip the order
    [~, th2] = solveangle(a, b, z2);
    th3 = solveangle(-a, -b, z3);
end

theta = [th1; th2; th3; gamma];

% compute velocities
if numel(theta1) > 1
    th1dot = theta1(2);
    s2 = sin(th2);
    c2 = cos(th2);
    s3 = sin(th3);
    c3 = cos(th3);
    s14 = sin(th1 - gamma);

    adot = L(1) * c1 * th1dot;
    bdot = -L(1) * s1 * th1dot;

    zdot = L(1) * L(4) * s14 * th1dot;
    z2dot = zdot / L(2);
    z3dot = zdot / L(3);

    d2 = (a * c2 - b * s2);
    d3 = (a * c3 - b * s3);

    % for theta2 => (a, b), theta3 => (-a, -b) and same for derivatives
    if abs(d2) < 1e-6
        th2dot = 0;
    else
        th2dot = -(adot * s2 + bdot * c2 + z2dot) / d2;
    end

    if abs(d3) < 1e-6
        % does this only happen for parallelogram configuration?
        th3dot = th1dot;
    else
        th3dot = -(adot * s3 + bdot * c3 - z3dot) / d3;
    end

    thetadot = [th1dot; th2dot; th3dot; 0];
else
    thetadot = [];
end

% compute accelerations
if numel(theta1) > 2
    th1ddot = theta1(3);

    addot = L(1) * (-s1 * th1dot^2 + c1 * th1ddot);
    bddot = -L(1) * (c1 * th1dot^2 + s1 * th1ddot);

    zddot = L(1) * L(4) * (c14 * th1dot^2 + s14 * th1ddot);
    z2ddot = zddot / L(2);
    z3ddot = zddot / L(3);

    % for theta2 => (a, b), theta3 => (-a, -b) and same for derivatives

    if abs(d2) < 1e-6
        th2ddot = 0;
    else
        th2ddot = -(addot * s2 + bddot * c2 ...
            + 2 * (adot * c2 - bdot * s2) * th2dot ...
            - (a * s2 + b * c2) * th2dot^2 + z2ddot) / d2;
    end

    if abs(d3) < 1e-6
        th3ddot = th1ddot;
    else
        th3ddot = -(addot * s3 + bddot * c3 ...
            + 2 * (adot * c3 - bdot * s3) * th3dot ...
            - (a * s3 + b * c3) * th3dot^2 - z3ddot) / d3;
    end

    thetaddot = [th1ddot; th2ddot; th3ddot; 0];
else
    thetaddot = [];
end
end