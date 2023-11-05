
classdef (Abstract) Collider < Element
    % A collider element primitive

    properties (Abstract,Constant)
        Code;
    end
    properties (SetAccess = private)
        IsTrigger = false;
        Cid;
    end

    methods (Abstract)
        % Provide a way to evaluate collision with
        % this and a second collider.
        [points] = TestCollision(transformA,colliderB,transformB);
    end

    methods
        function [this] = Collider()
            % CONSTRUCTOR - Base definition of colliders.

            % Create a random colliderId
            this.Cid = RandIntOfLength(6);
        end
        function [this] = SetTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
    end

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
        function [points] = FindSphereSphereCollisionPoints(sTransformA,sColliderA,sTransformB,sColliderB)
            % Find the collision points between two spheres.

            % Sanity check
            assert(isa(sTransformA,"Transform"),"Expecting a valid first transform object.");
            assert(sColliderA.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(isa(sTransformB,"Transform"),"Expecting a valid second transform object.");
            assert(sColliderB.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Separation axis
            collisionAxis = sTransformA.position - sTransformB.position;
            distance = norm(collisionAxis);
            normal = collisionAxis/distance;
            % Points furthest towards each other
            a = sTransformA.position + normal*sColliderA.Radius;
            b = sTransformB.position - normal*sColliderB.Radius;
            % The overlap distance
            depth = distance - (sColliderA.Radius + sColliderB.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = CollisionPoints();
                return;
            end

            %depth = depth/2;
            % Collision points
            points = CollisionPoints(a,b,normal,depth,isColliding);
        end
        function [points] = FindSpherePlaneCollisionPoints(sTransform,sCollider,pTransform,pCollider)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(isa(sTransform,"Transform"),"Expecting a valid first transform object.");
            assert(sCollider.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(isa(pTransform,"Transform"),"Expecting a valid second transform object.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Sphere properties
		    aCenter = sCollider.Center + sTransform.WorldPosition();
            aRadius = sCollider.Radius * sTransform.WorldScale();
            axis = pCollider.Normal/norm(pCollider.Normal);

            % Planar projection
            distance = dot(aCenter - pTransform.position,axis);
            
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
        function [points] = FindSphereOBBCollisionPoints(sTransform,sCollider,bTransform,bCollider)
            % Find the collision points between a sphere and an OBB box.

            % Sanity check
            assert(isa(bTransform,"Transform"),"Expecting a valid first transform object.");
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(isa(sTransform,"Transform"),"Expecting a valid second transform object.");
            assert(sCollider.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Seperation axis (box to sphere)
            axisRay = Ray.FromPoints(bTransform.position,sTransform.position);

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
            spherePointInBox = sTransform.position - sCollider.Radius*axisRay.Direction; 
            boxPointInSphere = bTransform.position + largestVertexProjection*axisRay.Direction;
            
            % Return the points
            points = CollisionPoints( ...
                spherePointInBox, ...
                boxPointInSphere, ...
                -axisRay.Direction,... % The ray is reversed
                toResolve, ...
                isColliding);
        end
        function [points] = FindPlaneOBBCollisionPoints(pTransform,pCollider,bTransform,bCollider)
            % Find the collision points between an OBB box and a plane.

            % Sanity check
            assert(isa(bTransform,"Transform"),"Expecting a valid first transform object.");
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(isa(pTransform,"Transform"),"Expecting a valid second transform object.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Create a ray using the plane origin
            axisRay = Ray.FromVector(pTransform.position,pCollider.Normal);    
            % Height of the box center above the plane
            centerToPlaneHeight = Ray.PointProjection(axisRay,bTransform.position);
            
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
            deepestPointOfBInA = pTransform.position - centerToPlaneHeight*axisRay.Direction;
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
        function [points] = FindOBBOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(isa(transformA,"Transform"),"Expecting a valid first transform object.");
            assert(colliderA.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(isa(transformB,"Transform"),"Expecting a valid second transform object.");
            assert(colliderB.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Create a ray using the plane origin
            axisRay = Ray.FromPoints(transformA.position,transformB.position); 
            reverseAxisRay = Ray.FromPoints(transformB.position,transformA.position); 
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