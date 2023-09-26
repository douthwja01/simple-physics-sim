
function [array] = RandZero(n,m)
% Creates an array or matrix of random numbers, uniformly distributed
% around zero (i.e in the range [-1:0:1]

if nargin < 2
    m = 1;
end

array = rand(n,m);
array = 2*(array - 0.5); % center about -0.5;0;0.5
end