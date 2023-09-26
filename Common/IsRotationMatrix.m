% Checks if a matrix is a valid rotation matrix.
function [flag] = IsRotationMatrix(R)

% Check shape and is populated
if ~IsSquare(R,3) || isempty(R)
    flag = false;
    return;
end

% Check symbolics
if (isa(R,"sym"))
    flag = true;
    return;
end

% This should hold true for all valid rotation matrices
n = eye(3) - R'*R;
flag = norm(n) < 1e-6;
end
