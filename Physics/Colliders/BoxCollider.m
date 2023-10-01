
classdef BoxCollider < Collider
    % A sphere collider primitive

    properties
        Type = ColliderCode.OBB; 
        Center = zeros(3,1);
    end

    methods
        function [points] = TestCollision(this,transformA,colliderB,transformB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Type
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = Collider.FindOBBSphereCollisionPoints(transformA,this,transformB,colliderB);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = Collider.FindOBBPlaneCollisionPoints(transformA,this,transformB,colliderB);
                 case ColliderCode.OBB
                    % The second collider is a plane
                    points = Collider.FindOBBOBBCollisionPoints(transformA,this,transformB,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end

        end
    end
end