
classdef (Abstract) Collider < Element
    % A collider element primitive

    properties (Abstract,Constant)
        Code;
    end
    properties
        IsTrigger = false;
        Cid;
    end
    properties (SetAccess = protected)
        AABB = AABB.empty;  % Limits structure
    end

    methods (Abstract)
        % Evaluate collision between this and another collider primitive.
        [points] = TestCollision(colliderB);
    end
    methods (Abstract, Access = protected)
        % Provide a means to recalculate the AABB primitive.
        [this] = RecalculateAABB(this);
    end

    % Main
    methods
        function [this] = Collider(entity)
            % CONSTRUCTOR - Base definition of colliders.

            % Allow immediate assignment to an entity
            if nargin > 1
                this.AssignEntity(entity);
            end

            % Create a random colliderId
            this.Cid = RandIntOfLength(6);
        end
        % Get/sets
        function set.IsTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
        function set.Cid(this,cid)
            assert(isnumeric(cid),"Expecting a valid integer cid.");
            this.Cid = int32(cid);
        end
    end

    % Callbacks
    methods %(Access = private)
        function [this] = OnCollision(this,collision)
            % When the on-collision event is triggered by the world.            

            fprintf("Collider '%s', collided with '%s'.\n", ...
                collision.ColliderA.Entity.Name, ...
                collision.ColliderB.Entity.Name);
        end
        function [this] = OnTrigger(this,trigger)
            % When the on-collision event is triggered by the world.

            fprintf("Trigger '%s', triggered by '%s'\n.", ...
                trigger.ColliderA.Entity.Name, ...
                trigger.ColliderB.Entity.Name);
        end
    end
    % Collider Utilities
    methods (Static)
        function [points] = FindSphereSphereCollisionPoints(sColliderA,sColliderB)
            % Find the collision points between two spheres.

            % Sanity check
            assert(sColliderA.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(sColliderB.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");
            
            % Pull out the world positions
            positionA = sColliderA.Transform.WorldPosition();
            positionB = sColliderB.Transform.WorldPosition();

            % Separation axis
            collisionAxis = positionA - positionB;
            distance = norm(collisionAxis);
            normal = collisionAxis/distance;
            % Points furthest towards each other
            a = positionA + normal*sColliderA.Radius;
            b = positionB - normal*sColliderB.Radius;
            % The overlap distance
            depth = distance - (sColliderA.Radius + sColliderB.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = CollisionPoints();
                return;
            end

            % Collision points
            points = CollisionPoints(a,b,normal,depth,isColliding);
        end
        function [points] = FindSpherePlaneCollisionPoints(sCollider,pCollider)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(sCollider.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            sTransform = sCollider.Transform;
            % Origin positions in the world
            pWorldPosition = pCollider.Transform.WorldPosition();

            % Sphere properties
		    aCenter = sCollider.Center + sTransform.WorldPosition();
            aRadius = sCollider.Radius * sTransform.WorldScale();
            axis = pCollider.Normal/norm(pCollider.Normal);

            % Planar projection
            distance = dot(aCenter - pWorldPosition,axis);
            
            % Is it colliding
            isColliding = ~(distance > aRadius);
            if ~isColliding
			    points = CollisionPoints();
                return;
            end
            % Calculate the scalar distance to be resolved
            toResolve = distance - aRadius;
		    sDepth = aCenter - axis * aRadius;
		    pDepth = aCenter - axis * toResolve;
            % Return the collision points
            points = CollisionPoints(pDepth,sDepth,axis,toResolve,isColliding);
        end     
        function [points] = FindSphereOBBCollisionPoints(sCollider,bCollider)
            % Find the collision points between a sphere and an OBB box.

            % Sanity check
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(sCollider.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Pull out the transforms
            sTransform = sCollider.Transform;
            bTransform = bCollider.Transform;
            % Origin positions in the world
            sWorldPosition = sTransform.WorldPosition();
            bWorldPosition = bTransform.WorldPosition();

            % Seperation axis (box to sphere)
            axisRay = Ray.FromPoints(bWorldPosition,sWorldPosition);

            % Get the collider mesh
            collisionMesh = bCollider.Mesh.TransformBy(bTransform.transform);
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
            toResolve = (largestVertexProjection + sCollider.Radius) - axisRay.Magnitude;
            % No collision is occuring
            isColliding = toResolve > 0;
            if ~isColliding
                points = CollisionPoints();
                return;
            end
            % The points along the ray
            spherePointInBox = sWorldPosition - sCollider.Radius*axisRay.Direction; 
            boxPointInSphere = bWorldPosition + largestVertexProjection*axisRay.Direction;
            
            % Return the points
            points = CollisionPoints( ...
                spherePointInBox, ...
                boxPointInSphere, ...
                -axisRay.Direction,... % The ray is reversed
                toResolve, ...
                isColliding);
        end
        function [points] = FindPlaneOBBCollisionPoints(pCollider,bCollider)
            % Find the collision points between an OBB box and a plane.

            % Sanity check
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            pTransform = pCollider.Transform;
            bTransform = bCollider.Transform;
            % Origin positions in the world
            pWorldPosition = pTransform.WorldPosition();
            bWorldPosition = bTransform.WorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromVector(pWorldPosition,pCollider.Normal);    
            % Height of the box center above the plane
            centerToPlaneHeight = Ray.PointProjection(axisRay,bWorldPosition);
            
            % Get the collider mesh
            collisionMesh = bCollider.Mesh.TransformBy(bTransform.transform);
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
                points = CollisionPoints();
                return;
            end
            % The penetrating vertex
            deepestPointOfAInB = collisionMesh.Vertices(minIndex,:)';
            % The point on the plane closet to the vertex
            deepestPointOfBInA = pWorldPosition - centerToPlaneHeight*axisRay.Direction;
            % +ve depth to be resolved
            toResolve = abs(smallestVertexProjection);

            % Create the points
            points = CollisionPoints( ...
                deepestPointOfAInB, ...
                deepestPointOfBInA, ...
                axisRay.Direction,...
                toResolve,...
                isColliding);
        end
        function [points] = FindOBBOBBCollisionPoints(colliderA,colliderB)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(colliderA.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(colliderB.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Pull out the transforms
            transformA = colliderA.Transform;
            transformB = colliderB.Transform;
            % Origin positions in the world
            aWorldPosition = transformA.WorldPosition();
            bWorldPosition = transformB.WorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromPoints(aWorldPosition,bWorldPosition); 
            reverseAxisRay = Ray.FromPoints(bWorldPosition,aWorldPosition); 
            % Get the orientated collision meshes
            collisionMeshA = colliderA.Mesh.TransformBy(transformA.transform);
            collisionMeshB = colliderB.Mesh.TransformBy(transformB.transform);
            
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
                points = CollisionPoints();
                return;
            end
            % Get the points violating the opposing geometry
            deepestPointOfAInB = Ray.ProjectDistance(axisRay,maxAProjection);
            deepestPointOfBInA = Ray.ProjectDistance(reverseAxisRay,maxBProjection);

            % Create the points
            points = CollisionPoints( ...
                deepestPointOfAInB, ...
                deepestPointOfBInA, ...
                reverseAxisRay.Direction,...
                depth,...
                isColliding);
        end
    end
end