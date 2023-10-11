
classdef (Abstract) Collider < Element
    % A collider element primitive

    properties (Abstract,Constant)
        Code;
    end
    properties (SetAccess = private)
        IsTrigger = false;
    end

    methods (Abstract)
        % Provide a way to evaluate collision with
        % this and a second collider.
        [points] = TestCollision(transformA,colliderB,transformB);
    end

    methods
        function [this] = SetTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
    end

    events
        OnCollision; % We want to register a callback when a collision occurs.
    end

    % Collider Utilities
    methods (Static)
        % Sphere
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
            depth = depth/2;
            % Collision points
            points = CollisionPoints(a,b,normal,depth,~(depth > 0));
        end
        function [points] = FindSpherePlaneCollisionPoints(sTransform,sCollider,pTransform,pCollider)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(isa(sTransform,"Transform"),"Expecting a valid first transform object.");
            assert(sCollider.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(isa(pTransform,"Transform"),"Expecting a valid second transform object.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

% 		    aRadius = colliderB.Radius * major(bt.WorldScale());
% 		    normal  = rot_vec<_d>((colliderA.Normal/norm(colliderA.Normal)), at);
            
		    aCenter = sCollider.Center + sTransform.WorldPosition();
            aRadius = sCollider.Radius * sTransform.WorldScale();

            colliderNormal = pCollider.Normal/norm(pCollider.Normal);
            % The distance 
            h = aCenter - pTransform.position;
            distance = dot(h,colliderNormal);

            % Is it colliding
            isColliding = ~(distance > aRadius);

            if ~isColliding
			    points = CollisionPoints();
                return;
            end

            toResolve = -1; %aRadius - distance; 

		    aDeep = aCenter - colliderNormal * aRadius;
		    bDeep = aCenter - colliderNormal * toResolve;

            % Return the collision points
            points = CollisionPoints(aDeep,bDeep,colliderNormal,toResolve,isColliding);
        end
        function [points] = FindSphereOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between a sphere and an OBB box.
            
            points = CollisionPoints();
        end
        % Plane
        function [points] = FindPlaneSphereCollisionPoints(pTransform,pCollider,sTransform,sCollider)
            % Find the collision points between a plane and a sphere.

            % Sanity check
            assert(isa(sTransform,"Transform"),"Expecting a valid first transform object.");
            assert(sCollider.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(isa(pTransform,"Transform"),"Expecting a valid second transform object.");
            assert(pCollider.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

% 		    aRadius = colliderB.Radius * major(bt.WorldScale());
% 		    normal  = rot_vec<_d>((colliderA.Normal/norm(colliderA.Normal)), at);
            
		    aCenter = sCollider.Center + sTransform.WorldPosition();
            aRadius = sCollider.Radius * sTransform.WorldScale();

            colliderNormal = pCollider.Normal/norm(pCollider.Normal);
            % The distance 
            h = aCenter - pTransform.position;
            distance = dot(h,colliderNormal);

            % Is it colliding
            isColliding = ~(distance > aRadius);

            if ~isColliding
			    points = CollisionPoints();
                return;
            end

            toResolve = -1; %aRadius - distance; 

		    aDeep = aCenter - colliderNormal * aRadius;
		    bDeep = aCenter - colliderNormal * toResolve;

            % Return the collision points
            points = CollisionPoints(aDeep,bDeep,colliderNormal,toResolve,isColliding);
        end
        function [points] = FindPlaneOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between a plane and an OBB box.

            points = CollisionPoints();
        end
        % OBB (Box)
        function [points] = FindOBBSphereCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between an OBB box and a sphere.

            points = CollisionPoints();
        end
        function [points] = FindOBBPlaneCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between an OBB box and a plane.

            points = CollisionPoints();
        end
        function [points] = FindOBBOBBCollisionPoints(transformA,colliderA,transformB,colliderB)
            % Find the collision points between two OBB boxes.

            points = CollisionPoints();
        end
    end
end