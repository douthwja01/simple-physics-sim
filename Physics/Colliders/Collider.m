
classdef (Abstract) Collider < Element
    % A collider element primitive

    properties (Abstract,Constant)
        Type;
    end
    properties (SetAccess = private)
        IsTrigger = false;
    end

    methods (Abstract)
        % Provide a way to evaluate collision with 
        % this and a second collider.
        [points] = TestCollision(transformA,colliderB,transformB);
    end

    methods
        function [this] = SetTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
    end  

    events
        OnCollision; % We want to register a callback when a collision occurs.
    end

    % Collider Utilities
    methods (Static)
        % Sphere
        function [points] = FindSphereSphereCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between two spheres.

            % Collision resolutions
%             radialSum = colliderA.Radius + colliderB.Radius;
%             if distance < radialSum
%                 unitCollisionAxis = collisionAxis/distance;
%                 delta = radialSum - distance;
%                 transformA.Position = transformA.Position + 0.5*delta*unitCollisionAxis;
%                 transformB.Position = transformB.Position - 0.5*delta*unitCollisionAxis;
%             end

            % Separation axis
            collisionAxis = transformA.position - transformB.position;
            distance = norm(collisionAxis);
            normal = collisionAxis/distance;
            % Points furthest towards each other
            a = transformA.position + normal*colliderA.Radius;
            b = transformB.position - normal*colliderB.Radius;
            % The overlap distance
            depth = (colliderA.Radius + colliderB.Radius) - distance;
            % Collision points
            points = CollisionPoints(a,b,normal,depth);
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