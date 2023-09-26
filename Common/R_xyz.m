
function [R] = R_xyz(x,y,z,order)
% This function is designed to provide a range of rotation matrices defined
% by successive x, y, z rotations or a vector x = [xa,ya,za]. 

% Usages:
% R = R([1,2,3])
% R = R([1,2,3],"xyz")
% R = R(1,2,3)
% R = R(1,2,3,"xyz")

defaultOrder = "zyx";

% Variable input mode
switch nargin
    case 4
        % Do nothing
    case 3
        order = defaultOrder;
    case 2
        assert(numel(x) == 3,"Expecting a 3 element rotation vector.");
        order = string(y);
        z = x(3);
        y = x(2);
        x = x(1);
    case 1
        assert(numel(x) == 3,"Expecting a 3 element rotation vector.");
        order = defaultOrder;
        z = x(3);
        y = x(2);
        x = x(1);
    otherwise
        error("Bad input arguments.");
end
% Calculate the rotation matrix
order = upper(order);
switch (order)
    case "XYZ"
        R = R_x(x)*R_y(y)*R_z(z);
    case "ZYX"
        R = R_z(z)*R_y(y)*R_x(x);
    otherwise
        error("Order %s not recognised",order);
end

end