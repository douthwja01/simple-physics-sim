classdef CollisionSphere < CollisionPrimitive
    properties
        Radius = 1;
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this sphere is colliding with a given point
            % primitive.

            % Distance between the sphere center and the point
            flag = ~(CollisionTools.Distance(this.Position,point) > this.Radius);
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this sphere is colliding with a given sphere
            % primitive.

            % Distance between the sphere centers
            distance = CollisionTools.Distance(this.Position,sphere.Position);
            % Check spherical separation greater than the sum of the radii.
            flag = ~(distance > (this.Radius + sphere.Radius)); 
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this sphere is colliding with a given aabb
            % primitive.


        end
        function [flag] = CheckPlane(this,plane)
            % Check if this sphere is colliding with a given plane
            % primitive.
            
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this sphere is colliding with a given obb
            % primitive.
            
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this sphere is colliding with a given capsule
            % primitive.
            flag = capsule.CheckSphere(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this sphere is colliding with a given triangle
            % primitive.

        end
        function [flag] = CheckRay(this,ray)
            % Check if this sphere is colliding with a given ray
            % primitive.
        end
        function [flag] = CheckLine(this,line)
            % Check if this sphere is colliding with a given line
            % primitive.
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this sphere is colliding with a given mesh
            % primitive.
            flag = mesh.CheckSphere(this);
        end
    end
    % Support methods
    methods
        function [closest] = GetClosestPoint(this,point)
            % Get the closest point on the sphere to a provided point.

            % Projection to that point
            delta = (point - this.Position);
            unit_delta = delta/norm(delta);
            % Project the radius in the direction of 'delta';
            closest = this.Position + unit_delta*this.Radius;
        end
        function [axisInterval] = GetAxisInterval(this,axis)

            
        end
    end
end