classdef (Abstract) Collider < Element
    % A simple collider-element primitive that denotes an object that can 
    % collide with another collider element in simulation.
    
    properties (Abstract,Constant)
        Code;
    end
    properties
        IsTrigger = false;
        Cid = uint32.empty;
        ShowEventsInConsole = false;
    end
    events
        Collided; 
    end

    % Main
    methods
        % Constructor
        function [this] = Collider(entity)
            % CONSTRUCTOR - Base definition of colliders.

            % Allow immediate assignment to an entity
            if nargin > 1
                this.AssignEntity(entity);
            end

            % Create a random colliderId
            this.Cid = uint32(RandIntOfLength(6));

            % Register for engine feedback
            addlistener(this,"Collided",@(src,evnt)this.OnColliderEvent(evnt));
        end
        % Get/sets
        function set.IsTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
        function set.Cid(this,cid)
            assert(isa(cid,"uint32"),"Expecting a 'uint32' cid code.");            
            this.Cid = cid;
        end
    end

    methods
        function [isColliding,points] = TestCollision(this,collider)
            % This function provides a general-purpose interface for
            % evaluating any collider against another.
            
            switch collider.Code
                case ColliderCode.None
                    isColliding = false;
                    points = ContactPoints.empty;
                case ColliderCode.Point
                    [isColliding,points] = this.CheckPoints(collider);
                case ColliderCode.Line
                    [isColliding,points] = this.CheckLine(collider);
                case ColliderCode.Ray
                    [isColliding,points] = this.CheckRay(collider);
                case ColliderCode.Sphere
                    [isColliding,points] = this.CheckSphere(collider);
                case ColliderCode.Plane
                    [isColliding,points] = this.CheckPlane(collider);
                case ColliderCode.Capsule
                    [isColliding,points] = this.CheckCapsule(collider);
                case ColliderCode.AABB
                    [isColliding,points] = this.CheckAABB(collider);
                case ColliderCode.OBB
                    [isColliding,points] = this.CheckOBB(collider);
                case ColliderCode.Triangle
                    [isColliding,points] = this.CheckTriangle(collider);
                case ColliderCode.Mesh
                    [isColliding,points] = this.CheckMesh(collider);
                otherwise
                    error("Collider code not recognised.");
            end
        end
    end

    % Further collider requirements
    methods (Abstract)
        % Provide a means to get the world AABB for the collider
        [aabb,cid] = GetWorldAABB(this);
        % Provide a means of visualising the collider
        [h] = Draw(container,colour);
    end

    %% Internals
    methods (Abstract)
        [isColliding,points] = CheckPoint(this,point);
        [isColliding,points] = CheckLine(this,line);
        [isColliding,points] = CheckRay(this,ray);
        [isColliding,points] = CheckSphere(this,sphere);
        [isColliding,points] = CheckPlane(this,plant);
        [isColliding,points] = CheckCapsule(this,capsule);
        [isColliding,points] = CheckAABB(this,aabb);
        [isColliding,points] = CheckOBB(this,obb);
        [isColliding,points] = CheckTriangle(this,triangle);
        [isColliding,points] = CheckMesh(this,mesh);
    end
    methods (Access = protected)
        % Event handles
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
end