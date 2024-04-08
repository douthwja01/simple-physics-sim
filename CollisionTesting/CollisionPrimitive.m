
classdef (Abstract) CollisionPrimitive < handle
    % The base primitive of all collision objects.

    properties
        Position = zeros(3,1);
    end
    % Core methods
    methods (Abstract)
        [flag] = CheckPoint(this,point);
        [flag] = CheckSphere(this,sphere);        
        [flag] = CheckAABB(this,aabb);
        [flag] = CheckPlane(this,plane);  
        [flag] = CheckOBB(this,obb);
        [flag] = CheckCapsule(this,capsule);
        [flag] = CheckTriangle(this,triangle);
        [flag] = CheckRay(this,ray);
        [flag] = CheckLine(this,line);
        [flag] = CheckMesh(this,mesh);
    end
    % Support methods
    methods (Abstract)
        % Get the interval of this shape along a given axis.
        [axisInterval] = GetAxisInterval(this,axis);
    end

    methods (Static)
        function [flag] = CheckAxisOverlap(shapeA,shapeB,axis)
            % We need to find the interval that shapeA and shape B take up
            % along 'axis'.
            
            % Get each shapes axis-interval
            aInterval = shapeA.GetAxisInterval(axis);
            bInterval = shapeB.GetAxisInterval(axis);
            
            % Check if there is an intersect/overlap
            flag = Interval.IsIntersecting(aInterval,bInterval);
        end
        function [proj] = FindScalarProjection(point, axis)
            % This function computes the scalar projection of a point on on
            % an axis (assumes axis is unitary)

            n = norm(axis);
            if n > 1
                axis = axis/n;
            end
            proj = dot(point, axis);
        end
        function [dist] = Distance(positionA,positionB)
            % Calculate the distance between two points in space.
            dist = norm(positionA - positionB);
        end
    end
end