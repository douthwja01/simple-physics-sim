
function [id] = RandIntOfLength(n)
% This function generates an integer number with 'n' characters.

% Sanity check
assert(isscalar(n),"Expecting a scalar integer number of data.");
assert(n > 0,"Expecting a length greater than 0.");

% Increment the id
id = 10^(n - 1);
% Add the base value to a random seed
id = id + randi(10^(n) - id,1);
end