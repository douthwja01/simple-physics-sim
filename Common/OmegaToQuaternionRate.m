function [dQ] = OmegaToQuaternionRate(q0)
% This function produces the quaternion rate coefficient matrix from a unit
% quaternion rotation to be multiplied by an angular rate vector "omega_b".

% Also called the:
% "Kinematic differential equation for the Euler parameters (quaternion)"

% Alternate formulation  (of [4x4])
% dQ = 0.5*[q0(1), -q0(2), -q0(3), -q0(4);
%     q0(2),  q0(1), -q0(4),  q0(3);
%     q0(3),  q0(4),  q0(1), -q0(2);
%     q0(4), -q0(3), -q0(2),  q0(1)];

% Rewritten to allow multiplication by omega_b directly (of [4x3])
dQ = 0.5*[-q0(2), -q0(3), -q0(4);
           q0(1), -q0(4),  q0(3);
           q0(4),  q0(1), -q0(2);
          -q0(3), -q0(2),  q0(1)];
end