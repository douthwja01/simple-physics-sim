
classdef OBBCollider < Collider
    % An Object-Aligned Bounding Box (OBB) collider primitive.
    % Distinct from mesh-colliders as its collider representation is
    % simpler than a standard mesh.

    properties (Constant)
        Code = ColliderCode.OBB;
    end
    properties
        Width = 1;
        Depth = 1;
        Height = 1;
    end

    % Main methods
    methods
        function [this] = OBBCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@Collider();
        end
    end
    %% Collision Pairing
    methods
        function [points] = CheckPoint(this,point)
            % Find the collision points between an obb and a point.
        end
        function [points] = CheckLine(this,line)
            % Find the collision points between an obb and a line.
        end
        function [points] = CheckRay(this,ray)
            % Find the collision points between an obb and a ray.
        end
        function [points] = CheckSphere(this,sphere)
            % Find the collision points between this OBB and a sphere.

            % TO FILL
        end
        function [points] = CheckPlane(this,plane)
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
        function [points] = CheckCapsule(this,capsule)
            % Find the collision points between an OBB and a capsule.
            points = capsule.CheckOBB(this);
        end
        function [points] = CheckAABB(this,aabb)
            % Find the collision points between an OBB and an AABB.

            % [TO FILL]
        end
        function [points] = CheckOBB(this,obb)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(this.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(obb.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Pull out the transforms
            transformA = this.Transform;
            transformB = obb.Transform;
            % Origin positions in the world
            aWorldPosition = transformA.GetWorldPosition();
            bWorldPosition = transformB.GetWorldPosition();

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
        function [points] = CheckTriangle(this,triangle)
            % Find the collision points between an obb and a triangle.
        end
        function [points] = CheckMesh(this,mesh)
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
            mesh = mesh.ScaleBy(this.Width,this.Height,this.Depth);
            % Transform the base mesh (by scale and position)
            mesh = mesh.TransformBy(this.Transform.GetWorldMatrix());
        end
        function [aabb] = GetWorldAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Get the mesh transformed in world coordinates
            mesh = this.GetWorldMesh();
            % Recompute AABB
            aabb = AABB.FromMesh(mesh);
            % Assign the owner's id
            aabb.Cid = this.Cid;
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
end