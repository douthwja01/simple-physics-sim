
function [flag] = IsRow(v,n)
% Default setting
flag = false;
% Get vector dimensions
[a,b] = size(v);
% Check it is a column
if a == 1 && nargin == 1
    flag = true;
    return;
end  
% Check if the specified length is in 'n'
if any(b == n)
    flag = true;
end
end