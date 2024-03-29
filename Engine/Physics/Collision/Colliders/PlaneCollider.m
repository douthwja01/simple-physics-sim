
classdef PlaneCollider < Collider
    % A plane collider primitive 

    properties (Constant)
        Code = ColliderCode.Plane;
    end
    properties
        Width = 1;
        Depth = 1;
        Thickness = 0.05;
        Normal = [0;0;1];
    end
    methods
        function [this] = PlaneCollider()
            % CONSTRUCTOR - Generate a plane-collider with a normal.

            % Create a collider
            [this] = this@Collider();   
        end
        function set.Normal(this,n)
            assert(IsColumn(n,3),"Expecting a normal [3x1].");
            this.Normal = n;
        end
    end
    % Collision Utilities
    methods
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is sphere
                    points = CollisionExtensions.FindSpherePlaneContactPoints(colliderB,this);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = CollisionExtensions.FindPlaneOBBContactPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
        function [aabb] = GetWorldAABB(this)
            % This function is called when the normal vector is changed in
            % order to update the mesh that defines the plane to match the
            % new normal.
        
            % Recompute AABB
            w = this.Width;
            d = this.Depth;
            t = this.Thickness;
            aabb = AABB(0.5*[-w,w],0.5*[-d,d],[0,-t]);

            % Position & scale
            p = this.Transform.GetWorldPosition();
            scale = this.Transform.GetWorldScale();
            % Offset the aabb by the sphere's world position
            aabb = aabb * scale + p;
            % Assign the parent
            aabb.Parent = this;
        end
    end
end