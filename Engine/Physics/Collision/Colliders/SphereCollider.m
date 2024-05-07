classdef SphereCollider < Collider
    % A sphere collider primitive

    properties (Constant)
        Code = ColliderCode.Sphere;
    end
    properties
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
    % Legacy
    methods
        function [aabb] = GetWorldAABB(this)
            % Overrided needed, sphere colliders do not need a mesh
            % representation as a unitary radius is valid. However, for
            % AABB comparison, a primitive must be constructed any way to
            % represent this radius.

            % Recompute AABB
            r = this.Radius;
            a = AABBCollider([-r,r],[-r,r],[-r,r]);

            % Position & scale
%             p = this.Transform.GetWorldPosition();
%             scale = this.Transform.GetWorldScale();
            p = this.Transform.Inertial.Position;
            scale = this.Transform.Inertial.Scale;

            % Offset the aabb by the sphere's world position
            aabb = a * scale + p;
            % Assign the owner's cid
            aabb.Cid = this.Cid;
        end
    end
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Check the collision points between a sphere and a point.
        end
        function [isColliding,points] = CheckLine(this,line)
            % Check the collision points between a sphere and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Check the collision points between a sphere and a ray.
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Check the collision points between a sphere and a triangle.
        end
        function [isColliding,points] = CheckSphere(this,sphere)
            % Check this collider against a second sphere collider

            % Sanity check
            assert(this.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(sphere.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Pull out the world positions
            positionA = this.Transform.GetWorldPosition();
            positionB = sphere.Transform.GetWorldPosition();

            % Separation axis
            seperationAxis = positionA - positionB;
            distance = norm(seperationAxis);
            unitSeperationAxis = seperationAxis/distance;
            % Points furthest towards each other
            a = positionA + unitSeperationAxis*this.Radius;
            b = positionB - unitSeperationAxis*sphere.Radius;
            % The overlap distance
            depth = distance - (this.Radius + sphere.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = ContactPoints();
                return;
            end

            % Collision points
            points = ContactPoints(a,b,unitSeperationAxis,depth,isColliding);
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            sWorldPosition = this.Transform.GetWorldPosition();
            % Origin positions in the world
            pWorldPosition = plane.Transform.GetWorldPosition();

            % Sphere properties
            aCenter = sWorldPosition;
            aRadius = this.Radius; % * sTransform.GetWorldScale();
            planeNormal = plane.Normal/norm(plane.Normal);

            % Planar projection
            distance = dot(sWorldPosition - pWorldPosition,planeNormal);

            % Is it colliding
            isColliding = ~(distance > aRadius);
            if ~isColliding
                points = ContactPoints();
                return;
            end
            % Calculate the scalar distance to be resolved
            toResolve = distance - aRadius;
            sDepth = aCenter - planeNormal * aRadius;
            pDepth = aCenter - planeNormal * toResolve;
            % Return the collision points
            points = ContactPoints(pDepth,sDepth,planeNormal,toResolve,isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)
            % Find the collision points between a sphere and a capsule.
            points = capsule.CheckSphere(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between a sphere and an AABB.
            points = aabb.CheckSphere(this);
        end
        function [isColliding,points] = CheckOBB(this,obb)
            % Find the collision points between a sphere and an OBB box.
            points = obb.CheckSphere(this);
        end
        function [isColliding,points] = CheckMesh(this,mesh)
            % Find the collision points between a sphere and a mesh.
            points = mesh.CheckSphere(this);
        end
    end
    %% Support
    methods (Access = protected)
        function [int] = GetAxisInterval(this,axis)
            % This function gets the projection of this collider on a given
            % axis.

            % Return the projection interval [TO-CHECK]
            int = Interval(-this.Radius,this.Radius);
        end
    end
end