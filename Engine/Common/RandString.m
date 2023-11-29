
function [randStr] = RandString(n)
% This function generates an arbitrary string 

% Sanity check
assert(isscalar(n),"Expecting a scalar string length.");
assert(n > 0,"Expecting a scalar string length.");

sample = 'abcdefghijklmnopqrstuwxyz1234567890';
indices = randi(strlength(sample),[1 n]);
randStr = sample(indices);
end