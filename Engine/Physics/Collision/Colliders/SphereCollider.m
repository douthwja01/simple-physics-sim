
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
            
            % Populate the equivalent AABB.
            this.RecalculateAABB();
        end
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = Collider.FindSphereSphereCollisionPoints(this,colliderB);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = Collider.FindSpherePlaneCollisionPoints(this,colliderB);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = Collider.FindSphereOBBCollisionPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
        % Get/sets
        function set.Radius(this,r)
            assert(isnumeric(r),"Expecting a numerical radius.");
            this.Radius = r;
            this.RecalculateAABB();
        end
    end
    methods (Access = protected)
        function [this] = RecalculateAABB(this)
            % Overrided needed, sphere colliders do not need a mesh
            % representation as a unitary radius is valid. However, for
            % AABB comparison, a primitive must be constructed any way to
            % represent this radius.

            % Recompute extents 
            r = this.Radius;
            xLims = this.Center + [-r,r];
            yLims = this.Center + [-r,r];
            zLims = this.Center + [-r,r];
            % Create the AABB
            this.AABB = AABB(xLims,yLims,zLims);
            this.AABB.Parent = this;
        end
    end
end