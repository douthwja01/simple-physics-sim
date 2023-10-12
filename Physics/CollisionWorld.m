
classdef CollisionWorld < handle
    % Collision world primitive responsible for managing the collision
    % properties of the simulation.

    properties (SetAccess = private)
        Colliders;
        Solvers;
        Callback = function_handle.empty;
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
            triggers = [];
            collisions = [];

            %entities = [this.Colliders.Entity];
            %uuids = [entities.Uuid];

            cache = [];
            
            % This function solves the inter-particle collisions (brute force)             
            for i = 1:numel(this.Colliders)
                collider_i = this.Colliders(i);
                uuid_i = collider_i.Entity.Uuid;

                for j = 1:numel(this.Colliders)
                    collider_j = this.Colliders(j);
                    uuid_j = collider_j.Entity.Uuid;

                    % Check if assessing self-collision
                    if (uuid_i == uuid_j)
                        continue
                    end

                    % Only check if both objects have colliders
                    if isempty(collider_i) || isempty(collider_j)
                        continue
                    end

                    tf_i = collider_i.Entity.GetElement("Transform");
                    tf_j = collider_j.Entity.GetElement("Transform");

                    if numel(tf_i) > 1 || numel(tf_j) > 1
                        warning("Shouldn't happen.");
                    end

                    % Test collisions with their respective colliders.
                    points = collider_i.TestCollision( ...
                        tf_i, ...
                        collider_j, ...
                        tf_j);

                    % If not colliding, skip
                    if ~points.IsColliding
                        continue
                    end

                    cache = vertcat(cache,[uuid_i,uuid_j]);

                    % Collision description
                    collision = Collision(collider_i,collider_j,points);

                    % If either are triggers
                    if collider_i.IsTrigger || collider_j.IsTrigger
                        triggers = vertcat(triggers,collision);
                    else
                        collisions = vertcat(collisions,collision);
                    end
                end
            end
            
            % No collisions occurred
            if isempty(collisions)
                return;
            end

            % Remove all symmetrical collisions
            [collisions] = this.RemoveSymmetricalCollisions(collisions,cache);

            % Resolve the collisions
            this.SolveCollisions(collisions,dt);
            % Issue collision and trigger event callbacks
            this.SendCollisionCallbacks(collisions,dt);
            this.SendCollisionCallbacks(triggers,dt);
        end
        % Managing collision objects
        function [this] = AddCollider(this,collider)
            if isempty(collider)
                return;
            end
            % Add a given object to the collision object list.
            this.Colliders = vertcat(this.Colliders,collider);
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
        function [this] = SolveCollisions(this,collisions,dt)
            % This function solves the set of identified collisions by
            % invoking the collision solvers.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid time step.");

            if isempty(collisions)
                return;
            end

            for i = 1:numel(this.Solvers)
                % Solve the collisions
                this.Solvers(i).Solve(collisions,dt);
            end
        end
        function [this] = SendCollisionCallbacks(this,collisions,dt)
            % Send the collision callbacks.

            for i = 1:numel(collisions)
                collision = collisions(i);
                % Notify the collider
                notify(collision.ColliderA,"OnCollision");
            end
        end
    end
    methods (Static,Access = private)
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