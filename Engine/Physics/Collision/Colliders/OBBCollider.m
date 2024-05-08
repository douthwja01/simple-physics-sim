
classdef OBBCollider < Collider
    % An Object-Aligned Bounding Box (OBB) collider primitive.
    % Distinct from mesh-colliders as its collider representation is
    % simpler than a standard mesh.

    properties (Constant)
        Code = ColliderCode.OBB;
    end
    properties
        Size = ones(3,1);
    end
    properties (SetAccess = private, Hidden)
        ImaginaryRadius = 1;
    end

    % Main methods
    methods
        function [this] = OBBCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@Collider();
        end
        % Get/sets
        function set.Size(this,s)
            assert(isscalar(s),"Expecting a size vector [3x1].");
            this.Size = s;
            this.RecalculateRadius();
        end
    end
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Find the collision points between an obb and a point.
        end
        function [isColliding,points] = CheckLine(this,line)
            % Find the collision points between an obb and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Find the collision points between an obb and a ray.
        end
        function [isColliding,points] = CheckSphere(this,sphere)
            % Find the collision points between this OBB and a sphere.

            % Containers
            isColliding = false;
            boxPosition = this.Transform.Inertial.Position;
            spherePosition = sphere.Transform.Inertial.Position;

            relativePosition = spherePosition - boxPosition;
            distance = norm(relativePosition);
            % Simple spherical check first
            if distance > (this.ImaginaryRadius + sphere.Radius)
                points = ContactPoints();
                return;
            end

            % Transform the sphere point into the local point
            localSpherePosition = this.Transform.InverseTransformPoint(spherePosition);

            % Calculate the closest point on the OBB to the sphere center
            dimSizes = this.Size/2;
            localClosestPoint = zeros(3,1);
            localClosestPoint(1) = Clamp([-dimSizes(1),dimSizes(1)],localSpherePosition(1));
            localClosestPoint(2) = Clamp([-dimSizes(2),dimSizes(2)],localSpherePosition(2));
            localClosestPoint(3) = Clamp([-dimSizes(3),dimSizes(3)],localSpherePosition(3));

            % Transform the local point back into world
            closestPoint = this.Transform.TransformPoint(localClosestPoint);

            collisionVector = closestPoint - spherePosition;
            collisionDistance = norm(collisionVector);
            collisionAxis = collisionVector/collisionDistance;
            
            % No collision if the distance to the nearest point is greater
            % than the radius
            isColliding = collisionDistance <= sphere.Radius;
            if ~isColliding
                points = ContactPoints();
                return;
            end

            % Calculate the collision points
            A = spherePosition + collisionAxis * (sphere.Radius  - collisionDistance);
            B = spherePosition - collisionAxis * (sphere.Radius  - collisionDistance);
            points = ContactPoints(A,B,collisionAxis,collisionDistance,isColliding);
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
            pWorldPosition = pTransform.Inertial.Position;
            bWorldPosition = bTransform.Inertial.Position;

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
            % Find the collision points between an OBB and a capsule.
            points = capsule.CheckOBB(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between an OBB and an AABB.

            rotationData = this.Transform.Rotation;

            axes = zeros(6,3);
            axes(1,:) = [1, 0, 0];
            axes(2,:) = [0, 1, 0];
            axes(3,:) = [0, 0, 1];
            axes(4,:) = rotationData(1,:);
            axes(5,:) = rotationData(2,:);
            axes(6,:) = rotationData(3,:);

            for i = 1:3
                for j = 4:6
                    crossAxes = cross(axes(i,:),axes(j,:));
                    axes = vertcat(axes,crossAxes);
                end
            end
            % Check for no overlap on each axis
            for i = 1:15
                if ~this.CheckAxisOverlap(this, aabb, axes(i,:))
                    flag = false;
                    return
                end
            end

            flag = true;
        end
        function [isColliding,points] = CheckOBB(this,obb)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(this.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(obb.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Origin positions in the world
            aWorldPosition = this.Transform.Inertial.Position;
            bWorldPosition = obb.Transform.Inertial.Position;

            % Create a ray using the plane origin
            axisRay = Ray.FromPoints(aWorldPosition,bWorldPosition);
            reverseAxisRay = Ray.FromPoints(bWorldPosition,aWorldPosition);
            % Get the orientated collision meshes
            collisionMeshA = this.GetWorldMesh();
            collisionMeshB = obb.GetWorldMesh();

            % Find the greatest projection of A on the axis
            vertexProjectionsA = zeros(collisionMeshA.NumberOfVertices,1);
            % Get the projections on the axis vector
            for i = 1:collisionMeshA.NumberOfVertices
                [vertexProjectionsA(i)] = Ray.PointProjection(axisRay,collisionMeshA.Vertices(i,:)');
            end
            % Maximum vertex extent from A towards B
            [maxAProjection] = max(vertexProjectionsA);

            % Find the greatest projection of B on the axis
            vertexProjectionsB = zeros(collisionMeshB.NumberOfVertices,1);
            % Get the projections on the axis vector
            for i = 1:collisionMeshB.NumberOfVertices
                [vertexProjectionsB(i)] = Ray.PointProjection(reverseAxisRay,collisionMeshB.Vertices(i,:)');
            end
            % Minimum vertex extent from B towards A (min w.r.t to ray direction)
            [maxBProjection] = max(vertexProjectionsB);

            % CHECK IF THIS EXCEEDS THE SEPARATION OF THE TWO OBJECTS
            depth = (maxAProjection + maxBProjection) - axisRay.Magnitude;
            % Is colliding
            isColliding = depth > 0;
            if ~isColliding
                points = ContactPoints();
                return;
            end
            % Get the points violating the opposing geometry
            deepestPointOfAInB = Ray.ProjectDistance(axisRay,maxAProjection);
            deepestPointOfBInA = Ray.ProjectDistance(reverseAxisRay,maxBProjection);

            % Create the points
            points = ContactPoints( ...
                deepestPointOfAInB, ...
                deepestPointOfBInA, ...
                reverseAxisRay.Direction,...
                depth,...
                isColliding);
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between an obb and a triangle.
        end
        function [isColliding,points] = CheckMesh(this,mesh)
            % Find the collision points between an OOB and a mesh.
            points = mesh.CheckOBB(this);
        end
    end
    % Support
    methods
        function [n] = GetWorldFaceNormals(this)
            % The standard normals
            n_unit = [
                1,0,0;
                0, 1,0;
                0,0, 1;
                0,0,-1;
                0,-1,0;
                -1,0,0];
            % Return the rotated normals
            n = this.Transform.GetWorldRotationMatrix()*n_unit;
        end
        function [mesh] = GetWorldMesh(this)
            % Calculate the boxes mesh in the world frame.

            mesh = MeshExtensions.UnitCube();
            % Transform the mesh by the box dimensions
            mesh = mesh.ScaleBy(this.Size);
            % Transform the base mesh (by scale and position)
            mesh = mesh.TransformBy(this.Transform.Inertial.GetMatrix());
        end
        function [aabb,cid] = GetWorldAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Get the mesh transformed in world coordinates
            r = this.ImaginaryRadius;
            unit_aabb = AABB([-r,r],[-r,r],[-r,r]);
            % Get the world properties
            so3 = this.Transform.Inertial;
            % Offset and scale the unit AABB
            aabb = so3.Position + so3.Scale*unit_aabb;
            % Assign the owner's id
            cid = this.Cid;
        end
        function [h] = Draw(this,container)
            % Draw the mesh collider

            % Input check
            if nargin < 2
                container = gca;
            end

            % get the transformed mesh
            mesh = this.GetWorldMesh();
            % Draw the collider to the container
            h = mesh.Draw(container,"r");
        end
    end
    methods (Access = protected)
        function [this] = RecalculateRadius(this)
            % Recalculate the Imaginary radius value
            this.ImaginaryRadius = sqrt(this.Size^2 + this.Height^2 + this.Depth^2);
        end
        function [axisInterval] = GetAxisInterval(this,axis)
            % This function creates an interval defining the projection of
            % this shape onto a given axis.

            % Ensure same size
            axis = axis';
            % Define vertex data from limits
            vertices = this.GetVertices();
            % We need to loop through these points and get the minimum and
            % maximums on the provided axis (axis is assumed to be
            % arbitrary)

            imin = inf; imax = -inf;
            for i = 1:size(vertices,1)
                projection = dot(axis,vertices);
                % If the projection is greater record it.
                if projection > imax
                    imax = projection;
                end
                % If the projection is smaller record it.
                if projection < imin
                    imin = projection;
                end
            end
            % Return an interval capturing it.
            axisInterval = Interval(imin,imax);
        end
    end
end