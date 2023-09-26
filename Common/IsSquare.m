
function [flag] = IsSquare(in,n)
flag = size(in,1) == size(in,2);
if nargin < 2
    return
end
flag = size(in,1) == n;
end