
classdef SphereCollider < Collider
    % A sphere collider primitive

    properties (Constant)
        Type = ColliderCode.Sphere; 
    end
    properties
        Center = zeros(3,1);
        Radius = 1;
    end

    methods
        function [points] = TestCollision(this,transformA,colliderB,transformB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Type
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = Collider.FindSphereSphereCollisionPoints(transformA,this,transformB,colliderB);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = Collider.FindSpherePlaneCollisionPoints(transformA,this,transformB,colliderB);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = Collider.FindSphereOBBCollisionPoints(transformA,this,transformB,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
    end
end