classdef CollisionExtensions
    %COLLISIONEXTENSIONS - An extension class providing access to a set of
    % collision utility methods used by multiple elements.

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
% 
%             % Pull out the transforms
%             transformA = boxA.Transform;
%             transformB = boxB.Transform;
%             % Origin positions in the world
%             aWorldPosition = transformA.GetWorldPosition();
%             bWorldPosition = transformB.GetWorldPosition();
% 
%             % Create a ray using the plane origin
%             axisRay = Ray.FromPoints(aWorldPosition,bWorldPosition);
%             reverseAxisRay = Ray.FromPoints(bWorldPosition,aWorldPosition);
%             % Get the orientated collision meshes
%             collisionMeshA = boxA.GetWorldMesh();
%             collisionMeshB = boxB.GetWorldMesh();
% 
%             % Find the greatest projection of A on the axis
%             vertexProjectionsA = zeros(collisionMeshA.NumberOfVertices,1);
%             % Get the projections on the axis vector
%             for i = 1:collisionMeshA.NumberOfVertices
%                 [vertexProjectionsA(i)] = Ray.PointProjection(axisRay,collisionMeshA.Vertices(i,:)');
%             end
%             % Maximum vertex extent from A towards B
%             [maxAProjection] = max(vertexProjectionsA);
% 
%             % Find the greatest projection of B on the axis
%             vertexProjectionsB = zeros(collisionMeshB.NumberOfVertices,1);
%             % Get the projections on the axis vector
%             for i = 1:collisionMeshB.NumberOfVertices
%                 [vertexProjectionsB(i)] = Ray.PointProjection(reverseAxisRay,collisionMeshB.Vertices(i,:)');
%             end
%             % Minimum vertex extent from B towards A (min w.r.t to ray direction)
%             [maxBProjection] = max(vertexProjectionsB);
% 
%             % CHECK IF THIS EXCEEDS THE SEPARATION OF THE TWO OBJECTS
%             depth = (maxAProjection + maxBProjection) - axisRay.Magnitude;
%             % Is colliding
%             isColliding = depth > 0;
%             if ~isColliding
%                 points = ContactPoints();
%                 return;
%             end
%             % Get the points violating the opposing geometry
%             deepestPointOfAInB = Ray.ProjectDistance(axisRay,maxAProjection);
%             deepestPointOfBInA = Ray.ProjectDistance(reverseAxisRay,maxBProjection);

            [isColliding,minOverlap,minOverlapAxis] = CollisionExtensions.CheckCollision(boxA,boxB);

            minOverlapAxis = minOverlapAxis'

            deepestPointOfAInB = minOverlapAxis*minOverlap + boxA.Transform.GetWorldPosition();
            deepestPointOfBInA = -minOverlapAxis*minOverlap + boxB.Transform.GetWorldPosition();

            % Create the points
            points = ContactPoints( ...
                deepestPointOfAInB, ...
                deepestPointOfBInA, ...
                minOverlapAxis,...
                minOverlap,...
                isColliding);

%             points = ContactPoints( ...
%                 deepestPointOfAInB, ...
%                 deepestPointOfBInA, ...
%                 reverseAxisRay.Direction,...
%                 depth,...
%                 isColliding);
        end

        %% -- TESTING

        %     	// References
        %     	// Getting the Right Axes to Test with
        %     	//https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat/
        %
        %         	//Unity Code, that nearly worked, but registered collisions incorrectly in some cases
        %         	//http://thegoldenmule.com/blog/2013/12/supercolliders-in-unity/

        % 	Vector3[] aAxes;
        % 	Vector3[] bAxes;
        % 	Vector3[] AllAxes;
        % 	Vector3[] aVertices;
        % 	Vector3[] bVertices;

        
        % 	List<Vector3> penetrationAxes;
        % 	List<float> penetrationAxesDistance;

        function [hasOverlap,minOverlap,minOverlapAxis] = CheckCollision(cubeA, cubeB)
            
            assert(isa(cubeA,"BoxCollider"),"Expecting a first valid box-collider input.");
            assert(isa(cubeB,"BoxCollider"),"Expecting a second valid box-collider input.");

            minOverlap = 0;
            minOverlapAxis = zeros(3,1);

            TA = cubeA.Transform.GetWorldMatrix();
            TB = cubeB.Transform.GetWorldMatrix();

            aMesh = cubeA.GetWorldMesh();
            aVertices = aMesh.Vertices;
            bMesh = cubeB.GetWorldMesh();
            bVertices = bMesh.Vertices;

            triad = [eye(3),zeros(3,1)];
            aAxes = triad*TA;
            bAxes = triad*TB;
            aAxes = aAxes(:,1:3);
            bAxes = bAxes(:,1:3);

            % All possible (separating) axes in a OBB-OBB interaction
            allAxes = zeros(15,3);
            % Local axes of a
            allAxes(1,:) = aAxes(:,1);  
            allAxes(2,:) = aAxes(:,2);
            allAxes(3,:) = aAxes(:,3);
            % Local axes of b
            allAxes(4,:) = bAxes(:,1);  
            allAxes(5,:) = bAxes(:,2);
            allAxes(6,:) = bAxes(:,3); 
            % A/B edge products
            allAxes(7,:) = cross(aAxes(:,1), bAxes(:,1)); 
            allAxes(8,:) = cross(aAxes(:,1), bAxes(:,2));
            allAxes(9,:) = cross(aAxes(:,1), bAxes(:,3));
            allAxes(10,:) = cross(aAxes(:,2), bAxes(:,1));
            allAxes(11,:) = cross(aAxes(:,2), bAxes(:,2));
            allAxes(12,:) = cross(aAxes(:,2), bAxes(:,3));
            allAxes(13,:) = cross(aAxes(:,3), bAxes(:,1));
            allAxes(14,:) = cross(aAxes(:,3), bAxes(:,2));
            allAxes(15,:) = cross(aAxes(:,3), bAxes(:,3));
                

%             penetrationAxes = zeros(3,1); %new List<Vector3>();
%             penetrationAxesDistance = 0; %new List<float>();


            [hasOverlap,minOverlap,minOverlapAxis,penAxes,penDistance] = CollisionExtensions.ProjectionHasOverlap(allAxes,bVertices,aVertices);
            if hasOverlap
                return
            end

            [hasOverlap,minOverlap,minOverlapAxis,penAxes,penDistance] = CollisionExtensions.ProjectionHasOverlap(allAxes,aVertices,bVertices);
            if hasOverlap
                return
            end
            % Penetration can be seen here, but its not reliable

		    fprintf("Min overlap %d : axis [%d,%d,%d]",minOverlap,minOverlapAxis(1),minOverlapAxis(2),minOverlapAxis(3));
        end

        
        function [hasOverlap,minOverlap,minOverlapAxis,penAxes,penDistance] = ProjectionHasOverlap(aAxes,aVertices,bVertices)
            % Detects whether or not there is overlap on all separating axes.

            % Output defaults
            hasOverlap = false;
            minOverlap = inf;
            minOverlapAxis = zeros(3,1);
            penAxes = [];
            penDistance = [];

            % Containers
            aAxesLength = size(aAxes,1);
            aVerticesLength = size(aVertices,1);
            bVerticesLength = size(bVertices,1);
            for i = 1:aAxesLength

                bProjMin = inf;
                aProjMin = inf;
                bProjMax = -inf;
                aProjMax = -inf;

                axis = aAxes(i,:);

                % Handles the cross product = {0,0,0} case
                if norm(axis) < 1E-5
                    hasOverlap = true;
                    return
                end

                % Test all 'b-vertices' on 'axis'
                for j = 1:bVerticesLength
                    val = CollisionExtensions.FindScalarProjection(bVertices(j,:), axis);
                    if val < bProjMin
                        bProjMin = val;
                    end
                    if val > bProjMax
                        bProjMax = val;
                    end
                end

                % Test all 'a-vertices' on 'axis'
                for j = 1:aVerticesLength
                    val = CollisionExtensions.FindScalarProjection(aVertices(j,:), axis);
                    if val < aProjMin
                        aProjMin = val;
                    end

                    if val > aProjMax
                        aProjMax = val;
                    end
                end

                overlap = CollisionExtensions.FindOverlap(aProjMin,aProjMax,bProjMin,bProjMax);

                if overlap < minOverlap
                    % If this overlap is less than previous overlaps
                    minOverlap = overlap;
                    minOverlapAxis = axis;
                    % Add the ax8s and distance to all penetrations
                    penAxes = vertcat(penAxes,axis);
                    penDistance = vertcat(penDistance,overlap);
                end

                % Separating Axis Found (Early Out)
                if overlap <= 0
                    hasOverlap = false;
                    return
                end
            end
            % A penetration has been found
        end
        function [overlap] = FindOverlap(astart, aend, bstart, bend)
            % Calculates the amount of overlap of two intervals.

            if astart < bstart
                if aend < bstart
                    overlap = 0;
                    return
                end
                overlap = aend - bstart;
                return ;
            end

            if bend < astart
                overlap = 0;
                return
            end

            overlap = bend - astart;
        end
    
        %% Other approach
        function [flag,collisionData] = CheckCollisionAxis (const Vector3 & axis , CollisionData & out_coldata )

            % // Overlap Test
            % // Points go:
            % // + - - - - - - - - - - - - -+
            % // + - - - - -| - - - - -+ 2 |
            % // | 1 | | |
            % // | + - - - - -| - - - - - - -+
            % // + - - - - - - - - - - -+
            % // A ------C- - -B ----- D
            % //
            % // IF A < C AND B > C ( Overlap in order object 1 -> object 2)
            % // IF C < A AND D > A ( Overlap in order object 2 -> object 1)

            %Vector3 min1 , min2 , max1 , max2 ;
            
            % Get the min /max vertices along the axis from shape1 and shape2
            
            cshapeA - > GetMinMaxVertexOnAxis ( axis , min1 , max1 );
            cshapeB - > GetMinMaxVertexOnAxis ( axis , min2 , max2 );
            
            A = FindScalarProjection( axis , min1 );
            B = FindScalarProjection( axis , max1 );
            C = FindScalarProjection( axis , min2 );
            D = FindScalarProjection( axis , max2 );
            
            % Overlap Test ( Order : Object 1 -> Object 2)
            if ( A <= C && B >= C )
                out_coldata . _normal = axis ;
                out_coldata . _penetration = C - B ;
                % Smallest overlap distance is between B- >C
                % Compute closest point on edge of the object
                out_coldata . _pointOnPlane =
                max1 + out_coldata . _normal * out_coldata . _penetration ;
            
                flag = true;
                return 
            end
            % Overlap Test ( Order : Object 2 -> Object 1)
            if ( C <= A && D >= A )
                out_coldata . _normal = - axis ;
                % Invert axis here so we can do all our resolution phase as
                % Object 1 -> Object 2
                out_coldata . _penetration = A - D ;
                % Smallest overlap distance is between D- >A
                % Compute closest point on edge of the object
                out_coldata . _pointOnPlane =
                min1 + out_coldata . _normal * out_coldata . _penetration ;

                % In
                flag = true;
                return
            end
            flag = false;

            end

    methods (Static)
        function [proj] = FindScalarProjection(point, axis)
            % This function computes the scalar projection of a point on on
            % an axis (assumes axis is unitary)

            n = norm(axis);
            if n > 1
                axis = axis/n;
            end
            proj = dot(point, axis);
        end
    end
end

