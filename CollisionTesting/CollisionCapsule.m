
classdef CollisionCapsule < CollisionPrimitive
    properties
        Height = 1;
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this capsule is colliding with a given point
            % primitive.

        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this capsule is colliding with a given sphere
            % primitive.

        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this capsule is colliding with a given aabb
            % primitive.

        end
        function [flag] = CheckPlane(this,plane)
            % Check if this capsule is colliding with a given plane
            % primitive.

        end
        function [flag] = CheckOBB(this,obb)
            % Check if this capsule is colliding with a given obb
            % primitive.

        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this capsule is colliding with a given capsule
            % primitive.

        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this capsule is colliding with a given triangle
            % primitive.

        end
        function [flag] = CheckRay(this,ray)
            % Check if this capsule is colliding with a given ray
            % primitive.

        end
        function [flag] = CheckLine(this,line)
            % Check if this capsule is colliding with a given line
            % primitive.

        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this capsule is colliding with a given mesh
            % primitive.

        end
    end
end