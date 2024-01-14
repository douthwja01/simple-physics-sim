function [flag] = NumelIsOdd(vec)
% This function just checks if the number elements in the vector is odd.

flag = mod(numel(vec),2) ~= 0;
end