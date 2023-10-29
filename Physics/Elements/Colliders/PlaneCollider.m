
classdef PlaneCollider < Collider
    % A plane collider primitive 

    properties (Constant)
        Code = ColliderCode.Plane;
    end
    properties
        Normal = [0;0;1];
    end
    methods
        function [this] = PlaneCollider()
            % CONSTRUCTOR - Generate a plane-collider with a normal.

            % Create a collider
            [this] = this@Collider();            
        end
        function [points] = TestCollision(this,transformA,colliderB,transformB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is sphere
                    points = Collider.FindSpherePlaneCollisionPoints(transformB,colliderB,transformA,this);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = Collider.FindPlaneOBBCollisionPoints(transformA,this,transformB,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end

        end
    end
end