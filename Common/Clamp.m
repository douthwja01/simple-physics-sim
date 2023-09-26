
function [value] = Clamp(range,value)

% Sanity check
assert(isnumeric(range) && isnumeric(value),"Range and value must be numeric.");
assert(numel(range) == 2,"Expecting a numeric range [1x2] or [1xn].");

minimum = min(range);
maximum = max(range);

% Min value rectify
if value < minimum
    value = minimum;
    return;
end

% Max value rectify
if value > maximum
    value = maximum;
    return;
end

end