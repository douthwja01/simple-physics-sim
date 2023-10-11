
classdef PlaneCollider < Collider
    % A plane collider primitive 

    properties (Constant)
        Code = ColliderCode.Plane;
    end
    properties
        Normal = [0;0;1];
        Distance = 0;
    end
    methods
        function [points] = TestCollision(this,transformA,colliderB,transformB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
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