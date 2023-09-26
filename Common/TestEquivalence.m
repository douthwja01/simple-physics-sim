
function [flag] = TestEquivalence(a,b,tolerance)

% Sanity check
assert(size(a,1) == size(b,1),"Expecting inputs of same dimensions.");
assert(size(a,2) == size(b,2),"Expecting inputs of same dimensions.");
assert(isnumeric(a) && isnumeric(b),"Expecting numeric inputs.")
if nargin < 3
    tolerance = 1E-15;
end

error = abs(single(b) - single(a));

flag = any(error(error > tolerance));

end