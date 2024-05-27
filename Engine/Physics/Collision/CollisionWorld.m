classdef CollisionWorld < World
    % Collision world primitive responsible for managing the collision
    % properties of the simulation.

    properties
        BroadPhaseDetector = SweepAndPruneBPCD();
    end
    properties (SetAccess = private)
        % Collidables
        Colliders;
        Resolvers = CollisionResolver.empty(0,1);
        % Simulation callbacks
        CollisionCallback = function_handle.empty;
        TriggerCallback = function_handle.empty;
    end

    % Simulation hooks (for subscribing programs)
    events (NotifyAccess = private)
        CollisionFeedback;
        TriggerFeedback;
    end

    %% Main
    methods
        % Constructor
        function [this] = CollisionWorld(worldSize)
            % COLLISIONWORLD - Construct an instance of the collision world
            % object.

            % Input check
            if nargin < 1
                worldSize = 10;
            end
            % Instantiate the world
            [this] = this@World(worldSize);
        end
        % Get/sets
        function set.BroadPhaseDetector(this,detector)
            % Add a given solver to the array of collision solvers.
            assert(isa(detector,"BroadPhaseCollisionDetection"),"Expecting a valid broad-phase collision solver.");
            this.BroadPhaseDetector = detector;
        end
    end

    %% Utilities
    methods
        % Resolve world Collisions
        function [this] = FindResolveCollisions(this,dt)
            % This function builds the lists of 'detected' collisions and
            % triggers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid numeric time-step.");

            % Reset containers
            triggers = Manifold.empty;
            collisions = Manifold.empty;

            % --- BROAD PHASE ---
            % This routine completes a broad-phase collision check
            [manifolds] = this.BroadPhaseDetector.ResolveManifolds(this.Colliders);

            % --- NARROW PHASE ---
            for i = 1:numel(manifolds)
                % Manifold data
                manifold  = manifolds(i);
                colliderA = manifold.ColliderA;
                colliderB = manifold.ColliderB;

                % Test collisions with their respective colliders
                [isColliding,points] = colliderA.TestCollision(colliderB);
                % If not colliding, skip
                if ~isColliding
                    continue
                end
    
                % Add the points
                manifold.Points = points;
    
                % If either are triggers
                if colliderA.IsTrigger || colliderB.IsTrigger
                    triggers = vertcat(triggers,manifold);
                else
                    collisions = vertcat(collisions,manifold);
                end
            end
            % --------------------

            % No collisions occurred
            if isempty(collisions)
                return;
            end

            % -- RESOLUTION PHASE ---
            % This routine computes the collision resolution
            this.CalculateCollisionResponses(collisions,dt);

            % --- EVENT GENERATION PHASE ---
            % Notify colliders events
            CollisionWorld.SendColliderEvents(triggers,collisions);
            % Notify world/engine events
            this.SendWorldEvents(triggers,collisions);
        end
        % Managing collision objects
        function [this] = AddCollider(this,collider)
            % Add a given collider to the collison world.

            % Sanity check
            if isempty(collider)
                return
            end
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
            
            % Add to set
            this.Colliders = vertcat(this.Colliders,collider);
        end
        function [this] = RemoveCollider(this,collider)
            % Remove a collider from the collision world.

            % Sanity check
            if isempty(collider)
                return
            end
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
            
            % Remove a given solver from the array of collisions solvers.
            this.Colliders = this.Colliders(this.Colliders ~= collider);
        end
        % Managing collision NarrowPhaseSolvers
        function [this] = AddSolver(this,solver)
            assert(isa(solver,"CollisionResolver"),"Expecting a valid narrow-phase collision solver.");

            % Add a given solver to the array of collision solvers.
            this.Resolvers = vertcat(this.Resolvers,solver);
        end
        function [this] = RemoveSolver(this,solver)
            % Delete the object from the world
            if isnumeric(solver)
                % Temporary index
                vec = 1:1:numel(this.Resolvers);
                % Remove the object
                this.Resolvers = this.Resolvers(vec ~= solver);
            else
                assert(isa(solver,"NarrowPhaseCollisionSolver"),"Expecting a valid 'Solver' object.");
                % Remove a given solver from the array of collisions solvers.
                this.Resolvers = this.Resolvers(this.Resolvers ~= solver);
            end
        end
        % Callbacks
        function [this] = SetCollisionCallback(this,eventHandle)
            % This function allows the setting of a collision callback
            % method.
            assert(isa(eventHandle,"function_handle"),"Expecting valid function handle.");
            this.CollisionCallback = addlistener(this,"Collision",eventHandle);
        end
        function [this] = SetTriggerCallback(this,eventHandle)
            % This function allows the setting of a collision callback
            % method.
            assert(isa(eventHandle,"function_handle"),"Expecting valid function handle.");
            this.TriggerCallback = addlistener(this,"Trigger",eventHandle);
        end
    end

    %% Internals
    methods (Access = private)
        function [this] = CalculateCollisionResponses(this,collisions,dt)
            % This function solves the set of identified collisions by
            % invoking the collision solvers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid time step.");

            if isempty(collisions) || numel(collisions) < 1
                return;
            end

            for i = 1:numel(this.Resolvers)
                % Solve the collisions
                this.Resolvers(i).Resolve(collisions,dt);
            end
        end
        % Send the callbacks for all collisions & triggers
        function SendWorldEvents(this,triggers,collisions)
            % Send the trigger callbacks to listeners of the
            % "TriggerFeedback" world-events.

            % Notify the triggers world events.
            for i = 1:numel(triggers)
                % Send to subscribers of the engine
                notify(this,"TriggerFeedback",triggers(i));
            end

            % Notify the collision world events.
            for i = 1:numel(collisions)
                % Send to subscribers of the engine.
                notify(this,"CollisionFeedback",collisions(i));
            end
        end
    end
    methods (Static, Access = private)
        function SendColliderEvents(triggers,collisions)
            % Notify the colliders of the events they are participating in.

            % Notify the triggers world events.

            for i = 1:numel(triggers)
                % Trigger instance
                manifold_i = triggers(i);

                % Send to the collider instance(s)
                if manifold_i.ColliderA.IsTrigger
                    % Notify
                    notify(manifold_i.ColliderA,"Collided",ColliderData(manifold_i.ColliderB));
                end
                if manifold_i.ColliderB.IsTrigger
                    % Notify
                    notify(manifold_i.ColliderB,"Collided",ColliderData(manifold_i.ColliderA));
                end
            end

            % Notify the collision world events.
            for i = 1:numel(collisions)
                % Collision instance
                manifold_i = collisions(i);
                % Send to the collider instance(s)
                if ~manifold_i.ColliderA.IsTrigger
                    % Notify
                    notify(manifold_i.ColliderA,"Collided",ColliderData(manifold_i.ColliderB));
                end
                if ~manifold_i.ColliderB.IsTrigger
                    % Notify
                    notify(manifold_i.ColliderB,"Collided",ColliderData(manifold_i.ColliderA));
                end
            end
        end
    end
end