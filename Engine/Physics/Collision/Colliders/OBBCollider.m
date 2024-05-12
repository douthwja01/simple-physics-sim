
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
            % Recalculate 
            this.RecalculateProperties();
        end
        % Get/sets
        function set.Size(this,s)
            assert(isscalar(s),"Expecting a size vector [3x1].");
            this.Size = s;
            this.RecalculateProperties();
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
        function [isColliding,points] = CheckSphere(this,sphere)            % [DONE]
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

            n = plane.normal;
            s03 = this.Transform.Inertial;
            orientation = s03.Quaternion.GetMatrix();
            position = s03.Position;

            plen = ...
                this.Size(1) * abs(dot(n,orientation(:,1))) + ...
                this.Size(2) * abs(dot(n,orientation(:,2))) + ...
                this.Size(3) * abs(dot(n,orientation(:,3)));

            dist = dot(n,position) - plane.Distance;
        
            isColliding = abs(dist) < plen;


%             % Pull out the transforms
%             pTransform = plane.Transform;
%             bTransform = this.Transform;
%             % Origin positions in the world
%             pWorldPosition = pTransform.Inertial.Position;
%             bWorldPosition = bTransform.Inertial.Position;
% 
%             % Create a ray using the plane origin
%             axisRay = Ray.FromVector(pWorldPosition,plane.Normal);
%             % Height of the box center above the plane
%             centerToPlaneHeight = Ray.PointProjection(axisRay,bWorldPosition);
% 
%             % Get the collider mesh
%             collisionMesh = this.GetWorldMesh();
%             vertexProjections = inf(collisionMesh.NumberOfVertices,1);
%             for i = 1:collisionMesh.NumberOfVertices
%                 % A given collision vertex
%                 coordinate = collisionMesh.Vertices(i,:)';
%                 % Its projection on the separation vector
%                 [vertexProjections(i)] = Ray.PointProjection(axisRay,coordinate);
%             end
% 
%             % Get the smallest projected distance
%             [smallestVertexProjection,minIndex] = min(vertexProjections);
%             % No collision is occuring
%             isColliding = smallestVertexProjection < 0;
% 
%             if ~isColliding
%                 points = ContactPoints();
%                 return;
%             end
%             % The penetrating vertex
%             deepestPointOfAInB = collisionMesh.Vertices(minIndex,:)';
%             % The point on the plane closet to the vertex
%             deepestPointOfBInA = pWorldPosition - centerToPlaneHeight*axisRay.Direction;
%             % +ve depth to be resolved
%             toResolve = abs(smallestVertexProjection);
% 
%             % Create the points
%             points = ContactPoints( ...
%                 deepestPointOfAInB, ...
%                 deepestPointOfBInA, ...
%                 axisRay.Direction,...
%                 toResolve,...
%                 isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)          % [DONE]
            % Find the collision points between an OBB and a capsule.
            points = capsule.CheckOBB(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)                % [DONE]
            % Find the collision points between an OBB and an AABB.

           % Find the collision points between two OBB boxes.

            % Sanity check
            assert(this.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(aabb.Code == ColliderCode.AABB,"Second collider must be a AABB collider.");

            % Variables
            isColliding = false;
            so3A = this.Transform.Inertial;
            so3B = aabb.Transform.Inertial;

            distance = norm(so3B.Position - so3A.Position);
            if distance > (this.ImaginaryRadius + obb.ImaginaryRadius)
                points = ContactPoints();
                return;
            end

            % Array of testable axes
            axesA = so3A.Rotation.GetMatrix()'; % [Should not be transposed?]
            axesB = eye(3);
            axes = [axesA;axesB];
            for i = 1:3 
                for  j = 4:6
                    axes = vertcat(axes,cross(axes(i,:),axes(j,:)));
                end
            end
            
            % Get the transformed vertices
            aVertices = this.GetVertices();
            bVertices = aabb.GetVertices();
            minSeparation = inf;
            minSeparationAxis = zeros(3,1);
            for i = 1:size(axes,1)
                queryAxis = [axes(i,1);axes(i,2);axes(i,3)];
                % If the axis is basically parallel
                axisLength = norm(queryAxis);
                if axisLength < 0.0001
                    continue
                end
                % Normalise the axis
                queryAxis = queryAxis/axisLength;
                % Project this obb on the axis
                [projectionA] = CollisionExtensions.GetAxisInterval(aVertices,queryAxis);
                [projectionB] = CollisionExtensions.GetAxisInterval(bVertices,queryAxis);
                
                % Intersection projections along the axis
                overlap = projectionA.Intersect(projectionB);
                % An axis exists with no overlap, no collision
                if isempty(overlap)
                    isColliding = false;
                    points = ContactPoints();
                    return;
                end

                % If the separation is less than recorded
                if overlap.Span < minSeparation
                    minAProjection = projectionA;
                    minBProjection = projectionB;
                    minSeparation = overlap.Span;
                    minSeparationAxis = queryAxis;
                end
            end
            
            % Calculate the collision points based on the minimum separation axis
            pointAinB = so3A.Position + (minAProjection.Span/2) * minSeparationAxis;
            pointBinA = so3B.Position - (minBProjection.Span/2) * minSeparationAxis;
            isColliding = true;
            % Construct the points
            points = ContactPoints( ...
                pointAinB, ...
                pointBinA, ...
                -minSeparationAxis, ...
                minSeparation, ...
                isColliding);
        end
        function [isColliding,points] = CheckOBB(this,obb)                  % [DONE]
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(this.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(obb.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Variables
            isColliding = false;
            so3A = this.Transform.Inertial;
            so3B = obb.Transform.Inertial;

            distance = norm(so3B.Position - so3A.Position);
            if distance > (this.ImaginaryRadius + obb.ImaginaryRadius)
                points = ContactPoints();
                return;
            end

            % Array of testable axes
            axesA = so3A.Rotation.GetMatrix()'; % [Should not be transposed?]
            axesB = so3B.Rotation.GetMatrix()';
            axes = [axesA;axesB];
            for i = 1:3 
                for  j = 4:6
                    axes = vertcat(axes,cross(axes(i,:),axes(j,:)));
                end
            end

            aVertices = this.GetVertices();
            bVertices = obb.GetVertices();
            minSeparation = inf;
            minSeparationAxis = zeros(3,1);
            for i = 1:size(axes,1)
                queryAxis = [axes(i,1);axes(i,2);axes(i,3)];
                % If the axis is basically parallel
                axisLength = norm(queryAxis);
                if axisLength < 0.0001
                    continue
                end
                % Normalise the axis
                queryAxis = queryAxis/axisLength;
                % Project this obb on the axis
                [projectionA] = CollisionExtensions.GetAxisInterval(aVertices,queryAxis);
                [projectionB] = CollisionExtensions.GetAxisInterval(bVertices,queryAxis);
                
                % Intersection projections along the axis
                overlap = projectionA.Intersect(projectionB);
                % An axis exists with no overlap, no collision
                if isempty(overlap)
                    isColliding = false;
                    points = ContactPoints();
                    return;
                end

                % If the separation is less than recorded
                if overlap.Span < minSeparation
                    minAProjection = projectionA;
                    minBProjection = projectionB;
                    minSeparation = overlap.Span;
                    minSeparationAxis = queryAxis;
                end

            end
            
            % Calculate the collision points based on the minimum separation axis
            pointAinB = so3A.Position + (minAProjection.Span/2) * minSeparationAxis;
            pointBinA = so3B.Position - (minBProjection.Span/2) * minSeparationAxis;

%             aLine = [so3A.Position,pointAinB]';
%             bLine = [so3B.Position,pointBinA]';
% 
%             plot3(gca,aLine(:,1),aLine(:,2),aLine(:,3),"r","LineWidth",2);
%             plot3(gca,bLine(:,1),bLine(:,2),bLine(:,3),"b","LineWidth",2);
% 
%             plot3(gca,pointAinB(1),pointAinB(2),pointAinB(3),"r^");
%             plot3(gca,pointBinA(1),pointBinA(2),pointBinA(3),"b^");

            isColliding = true;
            % Construct the points
            points = ContactPoints( ...
                pointAinB, ...
                pointBinA, ...
                -minSeparationAxis, ...
                minSeparation, ...
                isColliding);
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between an obb and a triangle.
        end
        function [isColliding,points] = CheckMesh(this,mesh)                % [DONE]
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
            n = n_unit*this.Transform.Inertial.Rotation.GetMatrix();
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
        function [vertices] = GetVertices(this)

            maxExtents = this.Size/2;
            minExtents = -maxExtents;
            vertices = zeros(8,3);
            vertices(1,:) = [maxExtents(1),maxExtents(2),maxExtents(3)];
            vertices(2,:) = [maxExtents(1),maxExtents(2),minExtents(3)];
            vertices(3,:) = [maxExtents(1),minExtents(2),minExtents(3)];
            vertices(4,:) = [maxExtents(1),minExtents(2),maxExtents(3)];
            vertices(5,:) = [minExtents(1),minExtents(2),minExtents(3)];
            vertices(6,:) = [minExtents(1),minExtents(2),maxExtents(3)];
            vertices(7,:) = [minExtents(1),maxExtents(2),maxExtents(3)];
            vertices(8,:) = [minExtents(1),maxExtents(2),minExtents(3)];
            % Transform
            temp = [vertices,ones(8,1)];
            T = this.Transform.Inertial.GetMatrix();
            temp = T*temp';
            vertices = temp(1:3,:)';
        end
        function [this] = RecalculateProperties(this)
            % Recalculate the Imaginary radius value
            this.ImaginaryRadius = norm(this.Size/2);
        end
    end
end