
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
    % Legacy
    methods
        function [aabb] = GetWorldAABB(this)
            % This function is called when the normal vector is changed in
            % order to update the mesh that defines the plane to match the
            % new normal.

            % Recompute AABB
            w = this.Width;
            d = this.Depth;
            t = this.Thickness;
            aabb = AABBCollider(0.5*[-w,w],0.5*[-d,d],[0,-t]);

            % Position & scale
%             p = this.Transform.GetWorldPosition();
%             scale = this.Transform.GetWorldScale();

            p = this.Transform.Inertial.Position;
            scale = this.Transform.Inertial.Scale;

            % Offset the aabb by the sphere's world position
            aabb = aabb * scale + p;
            % Assign the parent
            aabb.Cid = this.Cid;
        end
    end

    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Find the collision points between the plane and a point.
        end
        function [isColliding,points] = CheckLine(this,line)
            % Find the collision points between the plane and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Find the collision points between the plane and a ray.
        end
        function [isColliding,points] = CheckSphere(this,sphere)
            % Find the collision points between this plane and a sphere.
            points = sphere.CheckPlane(this);
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collision points between this OBB box and a plane.

            % Sanity check
            assert(this.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            pTransform = plane.Transform;
            bTransform = this.Transform;
            % Origin positions in the world
            pWorldPosition = pTransform.GetWorldPosition();
            bWorldPosition = bTransform.GetWorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromVector(pWorldPosition,plane.Normal);
            % Height of the box center above the plane
            centerToPlaneHeight = Ray.PointProjection(axisRay,bWorldPosition);

            % Get the collider mesh
            collisionMesh = this.GetWorldMesh();
            vertexProjections = inf(collisionMesh.NumberOfVertices,1);
            for i = 1:collisionMesh.NumberOfVertices
                % A given collision vertex
                coordinate = collisionMesh.Vertices(i,:)';
                % Its projection on the separation vector
                [vertexProjections(i)] = Ray.PointProjection(axisRay,coordinate);
            end

            % Get the smallest projected distance
            [smallestVertexProjection,minIndex] = min(vertexProjections);
            % No collision is occuring
            isColliding = smallestVertexProjection < 0;

            if ~isColliding
                points = ContactPoints();
                return;
            end
            % The penetrating vertex
            deepestPointOfAInB = collisionMesh.Vertices(minIndex,:)';
            % The point on the plane closet to the vertex
            deepestPointOfBInA = pWorldPosition - centerToPlaneHeight*axisRay.Direction;
            % +ve depth to be resolved
            toResolve = abs(smallestVertexProjection);

            % Create the points
            points = ContactPoints( ...
                deepestPointOfAInB, ...
                deepestPointOfBInA, ...
                axisRay.Direction,...
                toResolve,...
                isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)
            % Find the collision points between a plane and a capsule.
            points = capsule.CheckPlane(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between a plane and an AABB.
            points = aabb.CheckPlane(this);
        end
        function [isColliding,points] = CheckOBB(this,obb)
            % Find the collision points between a plane and an obb.
            points = obb.CheckPlane(this);
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between the plane and a triangle.

            % [TO FILL]
        end
        function [isColliding,points] = CheckMesh(this,mesh)
            % Find the collision points between a plane and a mesh.
            points = mesh.CheckPlane(this);
        end
    end

    %% Support
    methods (Access = protected)
        function [int] = GetAxisInterval(this,axis)
            % This function gets the projection of this collider on a given
            % axis.


        end
    end
end