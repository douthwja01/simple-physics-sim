classdef Simulator < handle
    % SIMULATOR - The master simulator class.

    properties
        FixedTimeDelta = 0.01;  % The fixed physics step
        SampleRate = 0.1;       % Time between physics updates
        % Modules
        Graphics;
        % Contents
        WorldSize = 10;
        g = [0;0;-9.81];
    end
    properties (SetAccess = private)
        World;
        Entities = [];
    end
    properties (Access = private)
        IsStopped = false;
    end

    % Main
    methods
        function [this] = Simulator()
            % CONSTRUCTOR - Construct an instance of the time simulator 
            % class representing a singular simulation.

            % Ensure all paths are available (needed for first run)
            this.AddEnginePaths;   
            % Create the dynamics world
            this.World = DynamicsWorld(this.WorldSize);
            % Add impulse collision solver
            this.World.AddSolver(ImpulseCollisionResolver());
            % Create a graphics handler
            this.Graphics = MatlabFigureGraphics();
        end
        % Get/sets
        function [this] = Simulate(this,duration)
            % This function executes the simulation sequence

            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");
            assert(~isempty(this.World),"Expecting a valid physics-world object, something went wrong.");
            assert(this.FixedTimeDelta < this.SampleRate,"Input sample rate should not be greater than the fixed-physics rate.");
            
            % Initialise the physics world with substeps
            this.World.Initialise();
            % Initialise the graphics output
            this.Graphics.Initialise(this.WorldSize);
            % Callback registration
            addlistener(this.World,"CollisionFeedback",@(src,evnt)this.OnCollisionCallback(src,evnt));
            addlistener(this.World,"TriggerFeedback",@(src,evnt)this.OnTriggerCallback(src,evnt));

            % Update routine
            timer = tic;
            t_accu = 0; t_last = 0; t_elapsed = 0;
            while t_elapsed < duration && ~this.IsStopped
                % Compute loop time
                t_delta = toc(timer);
                timer = tic;
                % Cap the maximum delta
                t_delta = min([t_delta,this.SampleRate]);
                % Compute the time data
                t_last = t_last + t_delta;
                t_accu = t_accu + t_delta;
                
                fprintf("[t=%.2fs] Stepping.\n",t_elapsed);

                % Compute the samepl
                while t_accu > this.SampleRate
                    % Update physics
                    this.World.Step(this.FixedTimeDelta);
                    t_accu = t_accu - this.SampleRate;
                end
                % Update visuals
                this.Graphics.Update();
                % Integrate the time
                t_elapsed = t_elapsed + t_delta;
            end
        end
        % Add/remove the objects
        function [entities] = Find(this,property,value)
            % Find an entity in the simulator by a given property.

            % Sanity check
            assert(isstring(property),"Expecting a value property label.");

            entities = [];
            if numel(this.Entities) == 0
                return;
            end

            % Get the entities by that property
            entities = this.Entities([this.Entities.(property)] == value);
        end
        function [this] = Add(this,entity)
            % Add an entity to the simulator.

            % Sanity check
            assert(isa(entity,"Entity"),"Expecting a valid 'Entity'.");

            % Add the entity by its transform
            this.World.AddTransform(entity.Transform);

            % Add the entity by its transform
            this.World.AddTransform(entity.Transform);
            % Add collider
            this.World.AddCollider(entity.Collider);
            % Add Rigid-body
            this.World.AddRigidBody(entity.RigidBody);
            % Add renderer
            this.Graphics.AddRenderer(entity.Renderer);
            % Add to entity-list
            this.Entities = vertcat(this.Entities,entity);
        end
        function [this] = Remove(this,entity)
            % Delete the entity from the simulator

            % Sanity check
            assert(isa(entity,"Entity"),"Expecting a valid 'Entity' object.");
            % Remove renderer
            this.Graphics.RemoveRenderer(entity.Renderer);
            % Remove collider
            this.World.RemoveCollider(entity.Collider);
            % Remove Rigid-body
            this.World.RemoveRigidBody(entity.RigidBody);
            % Add the entity by its transform
            this.World.RemoveTransform(entity.Transform);

            % Delete the entity from the world
            if isnumeric(entity)
                % Temporary index
                vec = 1:1:numel(this.Entities);
                % Remove the object
                this.Entities = this.Entities(vec ~= entity);
            else
                assert(isa(entity,"Entity"),"Expecting a valid 'Entity' object.");
                % Remove the object from the set
                this.Entities = this.Entities(this.Object ~= entity);
            end
        end
    end

    %% Internals
    methods (Access = private)
        function [this] = AddEnginePaths(this)
            % Confirm all subdirectories are on the path.

            % Get the path to this file
            enginePath = fileparts(mfilename('fullpath'));
            utilsPath = strcat(enginePath,"\Utilities");
            repoPath = erase(enginePath,"\Engine");
            % Gaurantee the common is always added
            addpath(utilsPath);
            % If elements are not on the path, add them
            Path.AddAllSubfolders(repoPath);
        end
        function [this] = OnCollisionCallback(this,source,event)
            % Do nothing when a collision occurs.
        end
        function [this] = OnTriggerCallback(this,source,event)
            % Do nothing when a trigger occurs.
        end
        function [this] = OnKeyPressCallback(this,source,event)
            % This function is called on key press withthe active figure.
            % disp(event.Key);

            if strcmpi(event.Key,"escape")
                fprintf("Simulation Stopped.\n");
                this.IsStopped = true;
            else
                warning("Press 'escape' to stop the simulation.");
            end
        end
    end
end