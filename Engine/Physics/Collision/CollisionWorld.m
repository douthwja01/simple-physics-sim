
classdef CollisionWorld < handle
    % Collision world primitive responsible for managing the collision
    % properties of the simulation.

    properties (SetAccess = private)
        % Collidables
        Colliders;
        % Broad-phase
        BroadPhaseSolver = SweepAndPrune();
        NarrowPhaseSolvers = NarrowPhaseCollisionSolver.empty(0,1);
        % Variables
        Collisions = Manifold.empty;
        Triggers = Manifold.empty;
        % Simulation callbacks
        CollisionCallback = function_handle.empty;
        TriggerCallback = function_handle.empty;
    end
    % Simulation hooks (for subscribing programs)
    events (NotifyAccess = private)
        CollisionFeedback;
        TriggerFeedback;
    end

    % Main
    methods
        % Resolve world Collisions
        function [this] = ResolveCollisions(this,dt)
            % This function builds the lists of 'detected' collisions and
            % triggers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid numeric time-step.");

            % Reset containers
            this.Triggers = Manifold.empty;
            this.Collisions = Manifold.empty;

            % --- BROAD PHASE ---
            % This routine completes a broad-phase collision check
            [manifolds] = this.BroadPhaseSolver.ResolveManifolds(this.Colliders);

            % --- NARROW PHASE ---
            % This routine solves the inter-particle collisions (brute force)   
            for i = 1:numel(manifolds)
                manifold_i = manifolds(i);
                
                % Evaluate if there has been a collision for the pair.
                this.TestCollision( ...
                    manifold_i.ColliderA, ....
                    manifold_i.ColliderB);
            end
            % --------------------

            % No collisions occurred
            if isempty(this.Collisions)
                return;
            end

            % Resolve the collisions
            this.SolveCollisions(dt);

            % --- EVENT GENERATION PHASE ---
            % Notify colliders events
            this.SendColliderEvents();
            % Notify world/engine events
            this.SendWorldEvents();
        end
        % Managing collision objects
        function [this] = AddCollider(this,collider)
            % Add a given collider to the collison world.

            % Sanity check 
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
            % Add to set
            this.Colliders = vertcat(this.Colliders,collider);
        end
        function [this] = RemoveCollider(this,collider)
            % Remove a collider from the collision world.

            % Sanity check 
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
            % Remove a given solver from the array of collisions solvers.
            this.Colliders = this.Colliders(this.Colliders ~= collider);
        end
        % Broad-phase
        function set.BroadPhaseSolver(this,s)
            assert(isa(s,"BroadPhaseSolver"),"Expecting a valid broad-phase solver.");
            this.BroadPhaseSolver = s;
        end        
        % Managing collision NarrowPhaseSolvers
        function [this] = AddSolver(this,solver)
            assert(isa(solver,"NarrowPhaseCollisionSolver"),"Expecting a valid narrow-phase collision solver.");
            
            % Add a given solver to the array of collision solvers.
            this.NarrowPhaseSolvers = vertcat(this.NarrowPhaseSolvers,solver);
        end
        function [this] = DeleteSolver(this,solver)
            % Delete the object from the world
            if isnumeric(solver)
                % Temporary index
                vec = 1:1:numel(this.NarrowPhaseSolvers);
                % Remove the object
                this.NarrowPhaseSolvers = this.NarrowPhaseSolvers(vec ~= solver);
            else
                assert(isa(solver,"NarrowPhaseCollisionSolver"),"Expecting a valid 'Solver' object.");
                % Remove a given solver from the array of collisions solvers.
                this.NarrowPhaseSolvers = this.NarrowPhaseSolvers(this.NarrowPhaseSolvers ~= solver);
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
    
    % Utilties
    methods (Access = private)
        function [points] = TestCollision(this,collider_i,collider_j)
            % Evaluate an individual collision instance

            % Test collisions with their respective colliders.
            points = collider_i.TestCollision(collider_j);

            % If not colliding, skip
            if ~points.IsColliding
                return
            end

            % Collision description
            manifold = Manifold(collider_i,collider_j,points);
            % If either are triggers
            if collider_i.IsTrigger || collider_j.IsTrigger
                this.Triggers = vertcat(this.Triggers,manifold);
            else
                this.Collisions = vertcat(this.Collisions,manifold);
            end
        end
        function [this] = SolveCollisions(this,dt)
            % This function solves the set of identified collisions by
            % invoking the collision solvers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid time step.");

            if isempty(this.Collisions) || numel(this.Collisions) < 1
                return;
            end

            for i = 1:numel(this.NarrowPhaseSolvers)
                % Solve the collisions
                this.NarrowPhaseSolvers(i).Solve(this.Collisions,dt);
            end
        end
        % Send the callbacks for all collisions & triggers
        function SendWorldEvents(this)
            % Send the trigger callbacks to listeners of the
            % "TriggerFeedback" world-events.

            % Notify the triggers world events.
            for i = 1:numel(this.Triggers)
                % Send to subscribers of the engine
                notify(this,"TriggerFeedback",manifolthis.Triggers(i));
            end

            % Notify the collision world events.
            for i = 1:numel(this.Collisions)
                % Send to subscribers of the engine.
                notify(this,"CollisionFeedback",this.Collisions(i));
            end
        end
        function SendColliderEvents(this)
            % Notify the colliders of the events they are participating in. 

            % Notify the triggers world events.
            for i = 1:numel(this.Triggers)
                % Trigger instance
                manifold_i = this.Triggers(i);
                % Send to the collider instance(s)
                if manifold_i.ColliderA.IsTrigger
                    data = ColliderData(manifold_i.ColliderB);
                    % Notify
                    notify( ...
                        this.Triggers(i).ColliderA, ...
                        "Collided", ...
                        data);
                end
                if manifold_i.ColliderB.IsTrigger
                    data = ColliderData(manifold_i.ColliderA);
                    % Notify
                    notify(this.Triggers(i).ColliderB,"Collided",data);
                end
            end

            % Notify the collision world events.
            for i = 1:numel(this.Collisions)
                % Collision instance
                manifold_i = this.Collisions(i);
                
                % Send to the collider instance(s)
                if ~manifold_i.ColliderA.IsTrigger
                    % Extract data
                    data = ColliderData(manifold_i.ColliderB);
                    % Notify
                    notify(manifold_i.ColliderA,"Collided",data);
                end
                if ~manifold_i.ColliderB.IsTrigger
                    % Extract data
                    data = ColliderData(manifold_i.ColliderA);
                    % Notify
                    notify(manifold_i.ColliderB,"Collided",data);
                end
            end
        end
    end
end