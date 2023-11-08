
function [points] = CreateGrid(position,number,columns,scale)
% This function distributes a series of points on a 3D grid, defined but a
% number of columns a scale.

rows = round(number/columns);
points = zeros(3,number);
ind = 1;
for c = 1:columns
    for r = 1:rows
        offset = [(r-1)*scale;0;(c-1)*scale];
        points(:,ind) = position + offset;
        % Increment the index
        ind = ind + 1;
    end
end
end