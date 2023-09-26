
function [flag] = IsColumn(v,n)
flag = false;
if size(v,2) ~= 1
    return
end
if nargin == 2
    flag = size(v,1) == n;
    return 
end
flag = true;
end