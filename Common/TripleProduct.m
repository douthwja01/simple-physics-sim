function [n] = TripleProduct(a,b,c)
% Calculate norm of a face defined by three points.
vec = cross(b - a, c - a);
n = vec/norm(vec);
end