function [Rnorm] = NormaliseRotationMatrix(R)
% This function normalises a given rotation matrix

assert(IsSquare(R,3),"Expecting a valid rotation matrix [3x3].");

j = R(1:3,2);
k = R(1:3,3);
% Recalculate perpendicular assets
i = cross(j, k);         % N = O x A
j = cross(k, i);         % O = A x N
% Reassign rotation
Rnorm = [i/norm(i),j/norm(j),k/norm(k)];
end