
classdef PlaneCollider < Collider
    % A plane collider primitive 

    properties
        Type = ColliderCode.Plane;
        Plane = [1;0;0];
        Distance = 0;
    end
    methods
        function [points] = TestCollision(transformA,colliderB,transformB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Type
                case ColliderCode.Sphere
                    % The second collider is sphere
                    points = Collider.FindPlaneSphereCollisionPoints(transformA,this,transformB,colliderB);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = Collider.FindPlaneOBBCollisionPoints(transformA,this,transformB,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end

        end
    end
end