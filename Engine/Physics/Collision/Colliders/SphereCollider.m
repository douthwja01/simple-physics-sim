classdef SphereCollider < Collider
    % A sphere collider primitive

    properties (Constant)
        Code = ColliderCode.Sphere; 
    end
    properties
        Center = zeros(3,1);
        Radius = 1;
    end

    methods
        function [this] = SphereCollider()
            % CONSTRUCTOR - Sphere collider.
        end
        % Get/sets
        function set.Radius(this,r)
            assert(isnumeric(r),"Expecting a numerical radius.");
            this.Radius = r;
        end
    end
    % Collision Utilities
    methods 
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = CollisionExtensions.FindSphereSphereContactPoints(this,colliderB);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = CollisionExtensions.FindSpherePlaneContactPoints(this,colliderB);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = CollisionExtensions.FindSphereOBBContactPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
        function [aabb] = GetWorldAABB(this)
            % Overrided needed, sphere colliders do not need a mesh
            % representation as a unitary radius is valid. However, for
            % AABB comparison, a primitive must be constructed any way to
            % represent this radius.

            % Recompute AABB
            r = this.Radius;
            a = AABB([-r,r],[-r,r],[-r,r]);

            % Position & scale
            p = this.Transform.GetWorldPosition();
            scale = this.Transform.GetWorldScale();
            % Offset the aabb by the sphere's world position
            aabb = a * scale + p;
            % Assign the parent
            aabb.Parent = this;
        end
    end
end