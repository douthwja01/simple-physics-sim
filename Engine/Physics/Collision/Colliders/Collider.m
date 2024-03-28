classdef (Abstract) Collider < Element
    % A simple collider-element primitive that denotes an object that can 
    % collide with another collider element in simulation.
    
    properties (Abstract,Constant)
        Code;
    end
    properties
        IsTrigger = false;
        Cid = uint8.empty;
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

    % Further collider requirements
    methods (Abstract)
        % Evaluate collision between this and another collider primitive.
        [points] = TestCollision(colliderB);
        % Provide a means to get the world AABB for the collider
        [aabb] = GetWorldAABB(this);
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
end