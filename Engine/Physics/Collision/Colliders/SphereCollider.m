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
            % Assign the owner's cid
            aabb.Cid = this.Cid;
        end
    end
    methods (Static)

        function [axes] = GetCollisionAxes(otherObject )
            % There are infinite possible axes on a sphere so we MUST
            % handle it seperately . Luckily we can just get the closest point
            % on the opposite object to our centre and use that .

            dir = (otherObject.GetPosition() - Parent().GetPosition()).Normalise();

            p1 = Parent ().GetPosition ();
            p2 = otherObject.GetCollisionShape().GetClosestPoint( p1 );

            axes = push_back (( p1 - p2 ). Normalise ());
        end

        function [closest] = GetClosestPoint(radius, point )
            % Get the closest point on a sphere to a given point.

            % From the centers
            dir = point - Parent().GetPosition();
            % Unit
            unit_dir = dir.Normalise();
            % Project the point in the direction, by the radius value
            closest = Parent().GetPosition() + unit_dir*radius;
        end

        function [out_min,out_max] = GetMinMaxVertexOnAxis(position,axis,radius) 
            
            out_min = position - axis * radius;
            out_max = position + axis * radius;
        end

    end
end