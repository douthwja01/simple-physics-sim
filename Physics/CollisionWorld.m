
classdef CollisionWorld < handle
    % Collision world primitive responsible for managing the collision
    % properties of the simulation.

    properties (SetAccess = private)
        Colliders;
        Solvers;
        Callback = function_handle.empty;
        % Variables
        Collisions = Collision.empty;
        Triggers = Collision.empty;
    end

    % Main
    methods
        function [this] = CollisionWorld()
            % Collision world constructor.
            
            % Add a solver
            this.AddSolver(PositionSolver());
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
%             cache = [];
            
            % This function solves the inter-particle collisions (brute force)             
            for i = 1:numel(this.Colliders)
                collider_i = this.Colliders(i);
                uuid_i = collider_i.Cid;

                for j = i+1:numel(this.Colliders)
                    collider_j = this.Colliders(j);
                    uuid_j = collider_j.Cid;

                    % Check if assessing self-collision
                    if (uuid_i == uuid_j)
                        continue
                    end

                    % Get the transforms
                    transform_i = collider_i.Entity.GetElement("Transform");
                    transform_j = collider_j.Entity.GetElement("Transform");

                    % Evaluate if there has been a collision for the pair.
                    [points] = this.TestCollision( ...
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

            % Remove all symmetrical collisions
%             [collisions] = this.RemoveSymmetricalCollisions(collisions,cache);

            % Resolve the collisions
            this.SolveCollisions(dt);
            % Issue collision and trigger event callbacks
            this.SendCollisionCallbacks(this.Collisions,dt);
            this.SendTriggerCallbacks(this.Triggers,dt);
        end
        % Managing collision objects
        function [this] = AddCollider(this,collider)
            % Allow the addition of a collider element to the collision
            % world

            % Add the collision object from the world
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
            % Add a given object to the collision object list.
            if ~isempty(collider)
                this.Colliders = vertcat(this.Colliders,collider);
            end
        end
        function [this] = RemoveCollider(this,collider)
            if isempty(collider)
                return;
            end
            % Delete the collision object from the world
            assert(isa(collider,"Collider"),"Expecting a valid 'Collider' element.");
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
            this.Callback = eventHandle;
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

%             cache = vertcat(cache,[uuid_i,uuid_j]);

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

    end
    methods (Static,Access = private)
        function SendTriggerCallbacks(instances,dt)
            % Send the collision callbacks.
            
            % Notify the collider
            for i = 1:numel(instances)
                notify(instances(i).ColliderA,"OnTrigger");
            end
        end
        function SendCollisionCallbacks(instances,dt)
            % Send the collision callbacks.
            
            % Notify the collider
            for i = 1:numel(instances)
                notify(instances(i).ColliderA,"OnCollision");
            end
        end
        function [collisions] = RemoveSymmetricalCollisions(collisions,cache)
            % This function moves through all collisions and removes
            % symmetrical/duplicate incidences.

            % Remove duplicate (symmetrical checks)
            cacheSize = size(cache,1);
            logicals = true(cacheSize,1);
            for i = 1:cacheSize
                %pair = cache(i);
                for j = i+1:cacheSize
                    
                    if cache(j,1) ~= cache(i,2)
                        continue;
                    end
                    if cache(j,2) ~= cache(i,1)
                        continue;
                    end
                    % Flag the duplicate
                    logicals(j) = false;
                end
            end
            collisions = collisions(logicals);
        end
    end
end