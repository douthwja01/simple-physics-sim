classdef CollisionPoint < CollisionPrimitive
    % A class representing a collision point object.

    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Simply return if the points are identical.
            flag = this.Position == point.Position;
        end
        function [flag] = CheckSphere(this,sphere)
           % Check if a point is within the sphere primitive.
            flag = ~(CollisionPoint.Distance(this.Position,sphere.Position) > sphere.Radius);
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if the point lies within a give aabb primitive.
            flag = false;
            for i = 1:3
                if ~aabb.Bounds(i).Contains(this.Position(i))
                    return 
                end
            end
            % The point is within the AABB
            flag = true;
        end
        function [flag] = CheckPlane(this,plane)
            % Check if the point is colliding with a given plane primitive.
            ndot = dot(this.Position,plane.Normal);
            flag = ndot == plane.Distance;
        end
        function [flag] = CheckOBB(this,obb)
            % Check if the point is colliding with a given OBB primitive.
            flag = obb.CheckPoint(this);
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if a point and capsule are intersecting/colliding.
            flag = capsule.CheckPoint(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check for intersection between the point and a given triangle
            % primitive.
            
            % The relative vertex positions
            pa = triangle.A - this.position;
            pb = triangle.B - this.position;
            pc = triangle.C - this.position;
            
            normPBC = cross(pb,pc);
            normPCA = cross(pc,pa);
            normPAB = cross(pa,pb);
            % Normalise
            normPBC = normPBC/norm(normPBC);
            normPCA = normPCA/norm(normPCA);
            normPAB = normPAB/norm(normPAB);

            flag = false;
            % Check if either projection is zero (i.e on the plane)
            if dot(normPBC,normPCA) < 1 || dot(normPBC,normPAB) < 1
                return;
            end
            flag = true;
        end
        function [flag] = CheckRay(this,ray)
            % Check this point is colliding with a given ray.

            % Get the nearest point on the ray to the point.
            nearest = ray.NearestPoint(this.Position);
            % If the distance to the closest point is greater than zero
            if CollisionTools.DistanceTo(this.Position,nearest) ~= 0 
                flag = false;
                return;
            end
            flag = true;
        end
        function [flag] = CheckLine(this,line)
            % Check if this point is colliding with a line primitive.

            % Nearest point on the line to this given point
            nearest = line.NearestPoint(this.Position);
            % Is the distance non-zero
            flag = CollisionTools.DistanceTo(this.Position,nearest) == 0;
        end
        function [flag] = CheckMesh(this,mesh)
            % Check to see if this point is colliding with a given mesh
            % primitive.
            flag = mesh.CheckPoint(this);
        end
    end
end