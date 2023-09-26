function [W] = OmegaToEulerRates(eulers)
% This function assembles the expression relating euler-rates to the body
% axis rates.

% Inspired by: https://robotacademy.net.au/lesson/the-analytic-jacobian/

% Input check
if nargin < 1
    eulers = [sym('PHI_t','real');sym('THETA_t','real');sym('PSI_t','real')];
end

% Sanity check
assert(IsColumn(eulers,3),'Euler angles must be of the form [3x1]');
% Mapping from the euler rate axes to the omega axes
W = [1,               0,                 sin(eulers(2));
     0,  cos(eulers(1)), -cos(eulers(2))*sin(eulers(1));
     0   sin(eulers(1)),  cos(eulers(2))*cos(eulers(1))];                  % Maps omega = W*euler_dot
end