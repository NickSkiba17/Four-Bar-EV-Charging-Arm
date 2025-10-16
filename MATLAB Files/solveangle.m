function [sln1, sln2] = solveangle(a, b, c)
%SOLVEANGLE Returns the solutions to a*sin(theta) + b*cos(theta) + c = 0
%
%   See also JOINTANGLES

% A helpful resource: https://math.stackexchange.com/questions/213545/solving-trigonometric-equations-of-the-form-a-sin-x-b-cos-x-c

c = -c;
e = a^2 + b^2;

d = e - c^2;
if d < 0
    d = 0;
else
    d = sqrt(e - c^2);
end

S = (a * c - b * d) / e;
C = (b * c + a * d) / e;
sln1 = atan2(S, C);

S = (a * c + b * d) / e;
C = (b * c - a * d) / e;
sln2 = atan2(S, C);
end

% an alternative (the initial) approach from Mallik, Ghosh, and Dittrich
% the above code is simpler, handles special cases, and avoids the if/else.
% don't flip sign of input c!!!!
% c = -c; % if necessary
% if c == b
%     sln1 = -(b + c) / (2 * a); % need to also deal with a = 0
%     sln1 = 2 * atan(sln1);
%     sln2 = sln1 + 2 * pi;
% else
%     sln1 = (-a + sqrt(a^2 - c^2 + b^2)) / (c - b);
%     sln1 = 2 * atan(sln1);
% 
%     sln2 = (-a - sqrt(a^2 - c^2 + b^2)) / (c - b);
%     sln2 = 2 * atan(sln2);
% end