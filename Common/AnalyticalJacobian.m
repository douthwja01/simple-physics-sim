
% GET SUBSTITUTIONS FOR BODY AXIS RATES
function [Ja] = AnalyticalJacobian(rotationParams)
% This function generates the jacobian matrix that moves from the body axis
% rates [w_t;v_t] to kinematic Cartesian rates [eta_dot;x_dot] or
% [q_dot;p_dot].

% Inspired by: https://robotacademy.net.au/lesson/the-analytic-jacobian/

% Sanity check
if nargin < 1
    rotationParams = [sym('PHI_t','real');sym('THETA_t','real');sym('PSI_t','real')];
end
% Sanity checks
assert(IsColumn(rotationParams,3) || IsColumn(rotationParams,4),'Expecting either a Euler or quaternion rotation vector.');

% Construct the Jacobian matrix
switch numel(rotationParams) 
    case 3
        % Get the euler coefficient matrix
        Binv = OmegaToEulerRates(rotationParams);
        % To align with [eta_dot;x_dot]
        Ja = [Binv, zeros(3); zeros(3), eye(3)]; 
    case 4
        % Get the quaternion coefficient matrix
        Binv = OmegaToQuaternionRate(rotationParams);
        % To align with [q_dot;x_dot]
        Ja = [Binv, zeros(4,3); zeros(3), eye(3)];
    otherwise
        error("Expecting either a Euler or quaternion rotation.");
end
end