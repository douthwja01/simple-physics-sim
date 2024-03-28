classdef CollisionExtensions
    %COLLISIONEXTENSIONS - An extension class providing access to a set of
    % collision utility methods used by multiple elements.
    
    properties
        Property1
    end
    
    %% Collider Utilities
    methods (Static)
        function [points] = FindSphereSphereContactPoints(sphereA,sphereB)
            % Find the collision points between two spheres.

            % Sanity check
            assert(sphereA.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(sphereB.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");
            
            % Pull out the world positions
            positionA = sphereA.Transform.GetWorldPosition();
            positionB = sphereB.Transform.GetWorldPosition();

            % Separation axis
            seperationAxis = positionA - positionB;
            distance = norm(seperationAxis);
            unitSeperationAxis = seperationAxis/distance;
            % Points furthest towards each other
            a = positionA + unitSeperationAxis*sphereA.Radius;
            b = positionB - unitSeperationAxis*sphereB.Radius;
            % The overlap distance
            depth = distance - (sphereA.Radius + sphereB.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = ContactPoints();
                return;
            end

            % Collision points
            points = ContactPoints(a,b,unitSeperationAxis,depth,isColliding);
        end
        function [points] = FindSpherePlaneContactPoints(sphere,plane)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(sphere.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            sWorldPosition = sphere.Transform.GetWorldPosition();
            % Origin positions in the world
            pWorldPosition = plane.Transform.GetWorldPosition();

            % Sphere properties
		    aCenter = sWorldPosition;
            aRadius = sphere.Radius; % * sTransform.GetWorldScale();
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
        function [points] = FindSphereOBBContactPoints(sphere,box)
            % Find the collision points between a sphere and an OBB box.

            % Sanity check
            assert(box.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(sphere.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Origin positions in the world
            sWorldPosition = sphere.Transform.GetWorldPosition();
            bWorldPosition = box.Transform.GetWorldPosition();

            % Seperation axis (box to sphere)
            axisRay = Ray.FromPoints(bWorldPosition,sWorldPosition);

            % Get the collider mesh
            collisionMesh = box.GetWorldMesh();
            vertexProjections = inf(collisionMesh.NumberOfVertices,1);
            for i = 1:collisionMesh.NumberOfVertices
                % A given collision vertex
                coordinate = collisionMesh.Vertices(i,:)';
                % Its projection on the separation vector
                [vertexProjections(i)] = Ray.PointProjection(axisRay,coordinate);
            end
            % Get the smallest projected distance
            [largestVertexProjection] = max(vertexProjections);
            
            % Resolve 
            toResolve = (largestVertexProjection + sphere.Radius) - axisRay.Magnitude;
            % No collision is occuring
            isColliding = toResolve > 0;
            if ~isColliding
                points = ContactPoints();
                return;
            end
            % The points along the ray
            spherePointInBox = sWorldPosition - sphere.Radius*axisRay.Direction; 
            boxPointInSphere = bWorldPosition + largestVertexProjection*axisRay.Direction;
            
            % Return the points
            points = ContactPoints( ...
                spherePointInBox, ...
                boxPointInSphere, ...
                -axisRay.Direction,... % The ray is reversed
                toResolve, ...
                isColliding);
        end
        function [points] = FindPlaneOBBContactPoints(plane,box)
            % Find the collision points between an OBB box and a plane.

            % Sanity check
            assert(box.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            pTransform = plane.Transform;
            bTransform = box.Transform;
            % Origin positions in the world
            pWorldPosition = pTransform.GetWorldPosition();
            bWorldPosition = bTransform.GetWorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromVector(pWorldPosition,plane.Normal);    
            % Height of the box center above the plane
            centerToPlaneHeight = Ray.PointProjection(axisRay,bWorldPosition);
            
            % Get the collider mesh
            collisionMesh = box.GetWorldMesh();
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
        function [points] = FindOBBOBBContactPoints(boxA,boxB)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(boxA.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(boxB.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Pull out the transforms
            transformA = boxA.Transform;
            transformB = boxB.Transform;
            % Origin positions in the world
            aWorldPosition = transformA.GetWorldPosition();
            bWorldPosition = transformB.GetWorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromPoints(aWorldPosition,bWorldPosition); 
            reverseAxisRay = Ray.FromPoints(bWorldPosition,aWorldPosition); 
            % Get the orientated collision meshes
            collisionMeshA = boxA.GetWorldMesh();
            collisionMeshB = boxB.GetWorldMesh();
            
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
    end
end

