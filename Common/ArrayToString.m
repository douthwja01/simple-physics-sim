
function [output] = ArrayToString(array,delimiter)
% Converts an array of numerics into a string seperated by the provided
% delimiter.

if nargin < 2
    delimiter = ",";
end

[r,c] = size(array);

assert(r == 1 || c == 1,"Expecting an array in a single direction.");

output = "[";
for i = 1:length(array)
    if (i == 1)
        output = strcat(output,sprintf("%.2f",array(i)));
    else
        output = strcat(output,sprintf("%s%.2f",delimiter,array(i)));
    end
end
output = strcat(output,"]");
end