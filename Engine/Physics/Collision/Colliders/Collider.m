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
        function [points] = TestCollision(this,collider)
            % This function provides a general-purpose interface for
            % evaluating any collider against another.
            
            switch collider.Code
                case ColliderCode.None
                    points = ContactPoints.empty;
                case ColliderCode.Point
                    points = this.CheckPoints(collider);
                case ColliderCode.Line
                    points = this.CheckLine(collider);
                case ColliderCode.Ray
                    points = this.CheckRay(collider);
                case ColliderCode.Sphere
                    points = this.CheckSphere(collider);
                case ColliderCode.Plane
                    points = this.CheckPlane(collider);
                case ColliderCode.Capsule
                    points = this.CheckCapsule(collider);
                case ColliderCode.AABB
                    points = this.CheckAABB(collider);
                case ColliderCode.OBB
                    points = this.CheckOBB(collider);
                case ColliderCode.Triangle
                    points = this.CheckTriangle(collider);
                case ColliderCode.Mesh
                    points = this.CheckMesh(collider);
                otherwise
                    error("Collider code not recognised.");
            end
        end
    end

    % Further collider requirements
    methods (Abstract)
        % Evaluate collision between this and another collider primitive.
        %[points] = TestCollision(collider);
        % Provide a means to get the world AABB for the collider
        [aabb] = GetWorldAABB(this);
    end

    %% Internals
    methods (Abstract)
        [points] = CheckPoint(this,point);
        [points] = CheckLine(this,line);
        [points] = CheckRay(this,ray);
        [points] = CheckSphere(this,sphere);
        [points] = CheckPlane(this,plant);
        [points] = CheckCapsule(this,capsule);
        [points] = CheckAABB(this,aabb);
        [points] = CheckOBB(this,obb);
        [points] = CheckTriangle(this,triangle);
        [points] = CheckMesh(this,mesh);
    end
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