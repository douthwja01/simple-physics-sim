
classdef (Abstract) Collider
    % A collider primitive

    properties (Abstract)
        Type;
    end

    methods (Abstract)
        % Provide a way to evaluate collision with 
        % this and a second collider.
        [points] = TestCollision(transformA,colliderB,transformB);
    end

    methods (Static)
        % Sphere
        function [points] = FindSphereSphereCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between two spheres.
            
        end
        function [points] = FindSpherePlaneCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collisions points between a sphere and a plane.

        end
        function [points] = FindSphereOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between a sphere and an OBB box.

        end
        % Plane
        function [points] = FindPlaneSphereCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between a plane and a sphere.

        end
        function [points] = FindPlaneOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between a plane and an OBB box.

        end
        % OBB (Box)
        function [points] = FindOBBSphereCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between an OBB box and a sphere.

        end
        function [points] = FindOBBPlaneCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between an OBB box and a plane.

        end
        function [points] = FindOBBOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between two OBB boxes.

        end
    end
end