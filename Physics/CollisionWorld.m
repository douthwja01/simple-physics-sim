
classdef CollisionWorld < handle
    % Collision world primitive responsible for managing the collision
    % properties of the simulation.

    properties (SetAccess = private)
        Colliders;
        Solvers;
        % Variables
        Collisions = Collision.empty;
        Triggers = Collision.empty;
        % Simulation callbacks
        CollisionCallback = function_handle.empty;
        TriggerCallback = function_handle.empty;
    end

    % Main
    methods
        function [this] = CollisionWorld()
            % Collision world constructor.
            
            % Add a solver
            this.AddSolver(PositionSolver());
%             this.AddSolver(ImpulseSolver());

            % Internal event loop-backs
            addlistener(this,"CollisionFeedback",@(src,evnt)this.OnInternalCollisionLoopback(evnt));
            addlistener(this,"TriggerFeedback",@(src,evnt)this.OnInternalTriggerLoopback(evnt));
        end
        % Resolve world Collisions
        function [this] = ResolveCollisions(this,dt)
            % This function builds the lists of 'detected' collisions and
            % triggers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid numeric time-step.");

            % Collision container
            this.Triggers = Collision.empty;
            this.Collisions = Collision.empty;
            
            % This function solves the inter-particle collisions (brute force)             
            for i = 1:numel(this.Colliders)
                collider_i = this.Colliders(i);
 
                for j = i+1:numel(this.Colliders)
                    collider_j = this.Colliders(j);
 
                    % Check if assessing self-collision
                    if (collider_i.Cid == collider_j.Cid)
                        continue
                    end

                    % Get the transforms
                    transform_i = collider_i.Entity.GetElement("Transform");
                    transform_j = collider_j.Entity.GetElement("Transform");

                    % Evaluate if there has been a collision for the pair.
                    this.TestCollision( ...
                        transform_i, ...
                        collider_i, ...
                        transform_j, ...
                        collider_j);
                end
            end
            
            % No collisions occurred
            if isempty(this.Collisions)
                return;
            end

            % Resolve the collisions
            this.SolveCollisions(dt);
            % Issue collision and trigger event callbacks
            this.SendCollisionCallbacks(this.Collisions);
            this.SendTriggerCallbacks(this.Triggers);
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
            % Unregister world from collider feedback
%             removelistener
            % Remove a given solver from the array of collisions solvers.
            this.Colliders = this.Colliders(this.Colliders ~= collider);
        end
        % Managing collision Solvers
        function [this] = AddSolver(this,solver)
            % Add a given solver to the array of collision solvers.
            this.Solvers = vertcat(this.Solvers,solver);
        end
        function [this] = DeleteSolver(this,solver)
            % Delete the object from the world
            if isnumeric(solver)
                % Temporary index
                vec = 1:1:numel(this.Solvers);
                % Remove the object
                this.Solvers = this.Solvers(vec ~= solver);
            else
                assert(isa(solver,"Solver"),"Expecting a valid 'Solver' object.");
                % Remove a given solver from the array of collisions solvers.
                this.Solvers = this.Solvers(this.Solvers ~= solver);
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
        function [points] = TestCollision(this,transform_i,collider_i,transform_j,collider_j)
            % Evaluate an individual collision instance

            % Test collisions with their respective colliders.
            points = collider_i.TestCollision( ...
                transform_i, ...
                collider_j, ...
                transform_j);

            % If not colliding, skip
            if ~points.IsColliding
                return
            end

            % Collision description
            manifold = Collision(collider_i,collider_j,points);
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

            for i = 1:numel(this.Solvers)
                % Solve the collisions
                this.Solvers(i).Solve(this.Collisions,dt);
            end
        end
        % Send the callbacks for all collisions & triggers
        function SendTriggerCallbacks(this,instances)
            % Send the collision callbacks.

            % Notify the collider
            for i = 1:numel(instances)
                notify(this,"TriggerFeedback",instances(i));
            end
        end
        function SendCollisionCallbacks(this,instances)
            % Send the collision callbacks.
           
            % Notify the object's collider that it has collided
            for i = 1:numel(instances)
                notify(this,"CollisionFeedback",instances(i));
            end
        end
    end
    % Collision and trigger loop-backs to colliders
    methods (Static, Access = private)
        function OnInternalTriggerLoopback(triggerEvent)
            % This method provides a local loop-back for all trigger
            % events that informs the object-collider (from the general
            % event) that they can enact their 'OnTrigger'.

%             fprintf("Trigger '%s', triggered by '%s'\n.", ...
%                 triggerEvent.ColliderA.Entity.Name, ...
%                 triggerEvent.ColliderB.Entity.Name);

            % Call the corresponding (local) collision callback
            triggerEvent.ColliderA.OnTrigger(triggerEvent);
        end
        function OnInternalCollisionLoopback(collisionEvent)
            % This method provides a local loop-back for all collision
            % events that informs the object-collider (from the general
            % event) that they can enact their 'OnCollision'.

%             fprintf("Object '%s' collided with '%s'.\n", ...
%                 collisionEvent.ColliderA.Entity.Name, ...
%                 collisionEvent.ColliderB.Entity.Name);

            % Call the corresponding (local) collision callback
            collisionEvent.ColliderA.OnCollision(collisionEvent);
        end
    end

    events (NotifyAccess = private)
        CollisionFeedback;
        TriggerFeedback;
    end
end