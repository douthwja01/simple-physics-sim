

function [wrench]    = RigidBodyForces(mass,inertia,twist,twistRate)    	% Rigidbody forces
% Inputs
% m - The rigid-body mass
% I - The inertia tensor
% t - The twist [w_t;v_t]
% Ouput
% Wi - The wrench excerted by the body [taui;fi]

% Sanity checks
assert(isscalar(mass),"Expecting a scalar mass variable.");
assert(IsSquare(inertia,3),"Expecting a [3x3] square, numeric or symbolic inertia tensor.");
assert(IsColumn(twist,6),"Expecting a [6x1] twist vector.");
assert(IsColumn(twistRate,6),"Expecting a [6x1] twist-rate vector.");

% Extract the angular rate from the twist
omega_t = twist(1:3);
% The inertial(mass) matrix
M = [inertia, zeros(3); zeros(3), mass*eye(3)];
% The centripetal & Coriolis matrix
C = [-Skew(inertia*omega_t),           zeros(3);
    zeros(3), mass*Skew(omega_t)];
% The net wrench expression
wrench = M*twistRate + C*twist;
end