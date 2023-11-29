
function [flag] = NumelIsEven(vec)
% This function just checks if the number elements in the vector is even.

flag = mod(numel(vec),2) == 0;
end