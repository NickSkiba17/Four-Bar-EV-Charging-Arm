function [c, ceq] = constraints(x)
%CONSTRAINTS returns vectors of constraints for use with fmincon
%   constraints(x) returns a pair of inequality c(x) <= 0 and equality
%   ceq(x) = 0 constraints for use  with fmincon.  The input x consists of
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
%   The constraints are
%        c(1) bounds on th1start to not exceed lowest input value
%        c(2) bounds on th1start to not exceed highest input value
%        c(3) bounds on th1end to not exceed lowest input value
%        c(4) bounds on th1end to not exceed highest input value
%        c(5) bounds on th1end and th1start to not exceed servo range
%        c(6) four-bar motion must remain inside frame in x-coords
%        c(7) four-bar motion must remain inside frame in x-coords
%        c(8) four-bar motion must remain inside frame in y-coords
%        c(9) four-bar motion must remain inside frame in y-coords
%        c(10) ee motion inside of frame must remain in frame in x-coords
%        c(11) ee motion inside of frame must remain in frame in x-coords
%        c(12) ee motion inside of frame must remain in frame in y-coords
%        c(13) ee motion inside of frame must remain in frame in y-coords
%        c(14) ee motion outside of frame must be between slit in y-coords
%        c(15) ee motion outside of frame must be between slit in y-coords
%        c(16) start and end input angles must lead to ee crossing the slit
%        c(17) the input torque cannot exceed min servo torque
%        c(18) the input torque cannot exceed max servo torque
%        c(19) the transmission angle must remain within suggested range
%        c(20) the transmission angle must remain within suggested range
%   and
%        ceq(1) ee at th1end must be at EV charge port location
%
%   See also COSTFUNCTION, OPTIMIZE, FMINCON.

% unpack decision variables
th1start = x(1);
th1end = x(2);
pos = x(3:6);
L = x(7:11);

% get various parameter values
th1bnds = inputrange(pos, L);
params = sharedparameters(pos, L);
framewidth = params.framewidth;
frameheight = params.frameheight;
evpos = params.evpos;
slit = params.slit;
servomax = params.servotorque;
servorange = params.servorange;
ta = params.transmissionangle;

% torque bounds, stay away from locking the mechanism: taubnds = inf
[taubnds, mubnds] = minmaxinputtorque([th1start, th1end], pos, L);

% position of ee at th1max
ee = endeffector(th1end, pos, L);
[eeinside, eeoutside] = minmaxee([th1start, th1end], pos, L);

% ee start and end positions relative to slit
eemin = endeffector(th1start, pos, L);
eemin = eemin(1, 2) - framewidth;
eemax = ee(1, 2) - framewidth;

% min/max position of 4-bar motion
[~, rodbnds] = minmaxposition([th1start, th1end], pos, L);

c = [
    % bounds on theta1 to avoid infeasible motions
    th1bnds(1) - th1start;
    th1start - th1bnds(2);
    th1bnds(1) - th1end;
    th1end - th1bnds(2);
    abs(th1end - th1start) - servorange;

    % motion of 4-bar (not including ee) must be inside frame
    0 - rodbnds(1);
    rodbnds(2) - framewidth;
    0 - rodbnds(3);
    rodbnds(4) - frameheight;

    % motion of ee must be inside frame until it reaches slit
    0 - eeinside(1);
    eeinside(2) - framewidth;
    0 - eeinside(3);
    eeinside(4) - frameheight;

    % motion of ee must be inside slit once outside of frame
    slit(2, 1) - eeoutside(3);
    eeoutside(4) - slit(2, 2);

    % make sure fmincon converges to motions that crosses the frame
    eemin * eemax;

    % bounds on input torque
    -servomax - taubnds(1);
    taubnds(2) - servomax;

    % bounds on transmission angle
    ta(1) - mubnds(1);
    mubnds(2) - ta(2)
    ];

% at th1max ee must be at target on ev
ceq = ee(:,2) - evpos;
end