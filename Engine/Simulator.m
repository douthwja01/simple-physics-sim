classdef Simulator < handle
    % SIMULATOR - The master simulator class.

    properties
        FixedTimeDelta = 0.01;  % Fixed physics step
        SampleRate = 0.25;      % Time between physics updates
        Physics;
        Graphics;
        % Contents
        WorldSize = 10;
        g = [0;0;-9.81];
    end
    properties (SetAccess = private)
        World
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

            % Parameters
            this.AddEnginePaths;   
            % Create a finite scene
            this.World = World(this.WorldSize);
            % Create a graphics handler
            this.Graphics = MatlabFigureGraphics();
            % Create a dynamics handler
            this.Physics = DynamicsWorld(this.World);
            % Add impulse collision solver
            this.Physics.AddSolver(ImpulseCollisionResolver());
        end
        % Get/sets
        function set.World(this,world)
            assert(isa(world,"World"),"Expecting a valid world object.");
            this.World = world;
        end
        % Interactions
        function [this] = Simulate(this,duration)
            % This function executes the simulation sequence

            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");
            assert(~isempty(this.World),"Expecting a valid 'DynamicsWorld' managing physics.");
            
            this.AddEnginePaths();

            % Initialise the physics world with substeps
            this.Physics.Initialise();
            % Initialise the graphics output
            this.Graphics.Initialise(this.WorldSize);

            addlistener(this.Physics,"CollisionFeedback",@(src,evnt)this.OnCollisionCallback(src,evnt));
            addlistener(this.Physics,"TriggerFeedback",@(src,evnt)this.OnTriggerCallback(src,evnt));

            % Update routine
            t_accu = 0; t_last = 0; t_elapsed = 0; timer = tic;
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

                % Compute the sample
                while t_accu > this.SampleRate
                    % Update physics
                    this.Physics.Step(this.FixedTimeDelta);
                    t_accu = t_accu - this.SampleRate;
                end
                % Update visuals
                this.Graphics.Update();
                % Integrate the time
                t_elapsed = t_elapsed + t_delta;
            end
        end
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
            % Add collider
            this.Physics.AddCollider(entity.Collider);
            % Add Rigid-body
            this.Physics.AddRigidBody(entity.RigidBody);
            % Add renderer
            this.Graphics.AddRenderer(entity.Renderer);
            % Add to entity-list
            this.Entities = vertcat(this.Entities,entity);
        end
        function [this] = Remove(this,entity)
            % Delete the entity from the simulator

            % Sanity check
            assert(isa(entity,"Entity"),"Expecting a valid 'Entity' object.");
            % Remove collider
            this.Physics.RemoveCollider(entity.Collider);
            % Remove Rigid-body
            this.Physics.RemoveRigidBody(entity.RigidBody);
            % Remove renderer
            this.Graphics.RemoveRenderer(entity.Renderer);
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
    % Internals
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
    end
    % Callbacks
    methods (Access = private)
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