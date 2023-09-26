% ROTATE A VECTOR ABOUT AN AXIS VECTOR (RODRIGUES)
function [v_rotated] = RodriguesRotation(v,axis,theta)
% u - Vector to be rotated
% n - Is the rotation axis
% Theta - The angle the vector is to be rotated through

assert(IsColumn(v,3),'Rotation vector must be of size [3x1].')
assert(IsColumn(axis,3),'The rotation axis must be of size [3x1]');
assert(numel(theta) == 1,'The rotation angle %.0f must be a scalar',theta);

[m,n] = size(v);
if (m ~= 3 && n ~= 3)
    error('input vector is/are not three dimensional')
end
if (size(v) ~= size(n))
    error('rotation vector v and axis k have different dimensions')
end

axis = axis/sqrt(axis(1)^2 + axis(2)^2 + axis(3)^2); % normalize rotation axis
No = numel(v)/3; % number of vectors in array
v_rotated = v; % initialize rotated vector array
if ( n == 3 )
    crosskv = v(1,:); % initialize cross product k and v with right dim.
    for i = 1:No
        crosskv(1) = axis(2)*v(i,3) - axis(3)*v(i,2);
        crosskv(2) = axis(3)*v(i,1) - axis(1)*v(i,3);
        crosskv(3) = axis(1)*v(i,2) - axis(2)*v(i,1);
        v_rotated(i,:) = cos(theta)*v(i,:) + (crosskv)*sin(theta)...
            + axis*(dot(axis,v(i,:)))*(1 - cos(theta));
    end
else % if m == 3 && n ~= 3
    crosskv = v(:,1); % initialize cross product k and v with right dim.
    for i = 1:No
        crosskv(1) = axis(2)*v(3,i) - axis(3)*v(2,i);
        crosskv(2) = axis(3)*v(1,i) - axis(1)*v(3,i);
        crosskv(3) = axis(1)*v(2,i) - axis(2)*v(1,i);
        v_rotated(:,i) = cos(theta)*v(:,i) + (crosskv)*sin(theta)...
            + axis*(dot(axis,v(:,i)))*(1 - cos(theta));
    end
end
end