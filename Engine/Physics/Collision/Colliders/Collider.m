
classdef (Abstract) Collider < Element
    % A collider element primitive

    properties (Abstract,Constant)
        Code;
    end
    properties
        IsTrigger = false;
        Cid;
        ShowEventsInConsole = false;
    end
    events
        Collided; 
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

            % Register for engine feedback
            addlistener(this,"Collided",@(src,evnt)this.OnColliderEvent(evnt));
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

    % Further collider requirements
    methods (Abstract)
        % Evaluate collision between this and another collider primitive.
        [points] = TestCollision(colliderB);
        % Provide a means to recalculate the AABB primitive.
        [aabb] = GetTransformedAABB(this);
    end

    %% Internals
    methods (Access = private)
        function [this] = OnColliderEvent(this,colliderData)
            % In the event of collision/trigger event raised, call the
            % associated feedbacks based on the collider configuration.

            % Throw the Trigger/collision call-back
            if this.IsTrigger
                this.OnTrigger(colliderData);
            else
                this.OnCollision(colliderData);
            end
        end
    end
    % Default methods (to override)
    methods (Access = protected)
        function [this] = OnCollision(this,colliderData)
            % When the on-collision event is triggered by the world.            

            % If requested, tell the world
            if this.ShowEventsInConsole
                fprintf("Collider '%s', collided with '%s'.\n", ...
                    colliderData.Source.Entity.Name, ...
                    colliderData.Collider.Entity.Name);
            end
        end
        function [this] = OnTrigger(this,colliderData)
            % When the on-trigger event is triggered by the world.

            % If requested, tell the world
            if this.ShowEventsInConsole
                fprintf("Trigger '%s', triggered by '%s'.\n", ...
                    colliderData.Source.Entity.Name, ...
                    colliderData.Collider.Entity.Name);
            end
        end
    end

    %% Collider Utilities
    methods (Static)
        function [points] = FindSphereSphereContactPoints(sColliderA,sColliderB)
            % Find the collision points between two spheres.

            % Sanity check
            assert(sColliderA.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(sColliderB.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");
            
            % Pull out the world positions
            positionA = sColliderA.Transformation.GetWorldPosition();
            positionB = sColliderB.Transformation.GetWorldPosition();

            % Separation axis
            seperationAxis = positionA - positionB;
            distance = norm(seperationAxis);
            unitSeperationAxis = seperationAxis/distance;
            % Points furthest towards each other
            a = positionA + unitSeperationAxis*sColliderA.Radius;
            b = positionB - unitSeperationAxis*sColliderB.Radius;
            % The overlap distance
            depth = distance - (sColliderA.Radius + sColliderB.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = ContactPoints();
                return;
            end

            % Collision points
            points = ContactPoints(a,b,unitSeperationAxis,depth,isColliding);
        end
        function [points] = FindSpherePlaneContactPoints(sCollider,pCollider)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(sCollider.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            sWorldPosition = sCollider.Transformation.GetWorldPosition();
            % Origin positions in the world
            pWorldPosition = pCollider.Transformation.GetWorldPosition();

            % Sphere properties
		    aCenter = sWorldPosition;
            aRadius = sCollider.Radius; % * sTransform.GetWorldScale();
            planeNormal = pCollider.Normal/norm(pCollider.Normal);

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
        function [points] = FindSphereOBBContactPoints(sCollider,bCollider)
            % Find the collision points between a sphere and an OBB box.

            % Sanity check
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(sCollider.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Origin positions in the world
            sWorldPosition = sCollider.Transformation.GetWorldPosition();
            bWorldPosition = bCollider.Transformation.GetWorldPosition();

            % Seperation axis (box to sphere)
            axisRay = Ray.FromPoints(bWorldPosition,sWorldPosition);

            % Get the collider mesh
            collisionMesh = bCollider.GetTransformedMesh();
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
                points = ContactPoints();
                return;
            end
            % The points along the ray
            spherePointInBox = sWorldPosition - sCollider.Radius*axisRay.Direction; 
            boxPointInSphere = bWorldPosition + largestVertexProjection*axisRay.Direction;
            
            % Return the points
            points = ContactPoints( ...
                spherePointInBox, ...
                boxPointInSphere, ...
                -axisRay.Direction,... % The ray is reversed
                toResolve, ...
                isColliding);
        end
        function [points] = FindPlaneOBBContactPoints(pCollider,bCollider)
            % Find the collision points between an OBB box and a plane.

            % Sanity check
            assert(bCollider.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            pTransform = pCollider.Transformation;
            bTransform = bCollider.Transformation;
            % Origin positions in the world
            pWorldPosition = pTransform.GetWorldPosition();
            bWorldPosition = bTransform.GetWorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromVector(pWorldPosition,pCollider.Normal);    
            % Height of the box center above the plane
            centerToPlaneHeight = Ray.PointProjection(axisRay,bWorldPosition);
            
            % Get the collider mesh
            collisionMesh = bCollider.GetTransformedMesh();
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
        function [points] = FindOBBOBBContactPoints(colliderA,colliderB)
            % Find the collision points between two OBB boxes.

            % Sanity check
            assert(colliderA.Code == ColliderCode.OBB,"First collider must be a box collider.");
            assert(colliderB.Code == ColliderCode.OBB,"Second collider must be a box collider.");

            % Pull out the transforms
            transformA = colliderA.Transformation;
            transformB = colliderB.Transformation;
            % Origin positions in the world
            aWorldPosition = transformA.GetWorldPosition();
            bWorldPosition = transformB.GetWorldPosition();

            % Create a ray using the plane origin
            axisRay = Ray.FromPoints(aWorldPosition,bWorldPosition); 
            reverseAxisRay = Ray.FromPoints(bWorldPosition,aWorldPosition); 
            % Get the orientated collision meshes
            collisionMeshA = colliderA.GetTransformedMesh();
            collisionMeshB = colliderB.GetTransformedMesh();
            
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