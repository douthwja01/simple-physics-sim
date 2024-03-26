classdef Simulator < handle
    % SIMULATOR - The master simulator class.

    properties
        TimeDelta = 0.01;
        Physics;
        % Contents
        g = [0;0;-9.81];
        WorldSize = 20;
    end
    properties (SetAccess = private)
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

            this.AddEnginePaths;

            % Create the dynamics world
            this.Physics = DynamicsWorld();
            % Add impulse collision solver
            this.Physics.AddSolver(ImpulseCollisionSolver());
        end
        function [this] = Simulate(this,duration)
            % This function executes the simulation sequence

            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");
            assert(~isempty(this.Physics),"Expecting a valid 'DynamicsWorld' managing physics.");
            
            this.AddEnginePaths();

            % Initialise the physics world with substeps
            this.Physics.Initialise();

            addlistener(this.Physics,"CollisionFeedback",@(src,evnt)this.OnCollisionCallback(src,evnt));
            addlistener(this.Physics,"TriggerFeedback",@(src,evnt)this.OnTriggerCallback(src,evnt));

            % Initialise the graphics
            [ax] = this.InitialiseGraphics();

            % Update routine
            time = 0;
            while (time < duration && ~this.IsStopped)
                fprintf("[t=%.2fs] Stepping.\n",time);
                % Update visuals
                this.UpdateGraphics(ax);
                % Update physics
                this.Physics.Step(this.TimeDelta);
                % Integrate the time
                time = time + this.TimeDelta;
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
            % Add collider
            cl = entity.Collider;
            if ~isempty(cl)
                this.Physics.AddCollider(cl);
            end
            % Add Rigid-body
            rb = entity.RigidBody;
            if ~isempty(rb)
                this.Physics.AddRigidBody(rb);
            end
            % Add to entity-list
            this.Entities = vertcat(this.Entities,entity);
        end
        function [this] = Remove(this,entity)
            % Delete the entity from the simulator

            % Sanity check
            assert(isa(entity,"Entity"),"Expecting a valid 'Entity' object.");
            % Add collider
            cl = entity.Collider;
            if ~isempty(cl)
                this.Physics.RemoveCollider(cl);
            end
            % Add Rigid-body
            rb = entity.RigidBody;
            if ~isempty(rb)
                this.Physics.RemoveRigidBody(rb);
            end

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
        function [ax,this] = InitialiseGraphics(this)
            % Draw the state of the world

            fig = figure("Name","Simulation");
            ax = axes(fig);
            hold on;
            set(ax,"View",[35,35])
            grid on;
            box on;
            axis equal;
            xlabel(ax,"X (m)");
            ylabel(ax,"Y (m)");
            zlabel(ax,"Z (m)");
            axisLimits = this.WorldSize/2;
            xlim(ax,[-axisLimits,axisLimits]);
            ylim(ax,[-axisLimits,axisLimits]);
            zlim(ax,[0,axisLimits]);
            % Register for key presses
            set(fig,'KeyPressFcn',@(src,evnt)OnKeyPressCallback(this,src,evnt));
        end
        function [this] = UpdateGraphics(this,ax)
            % This function updates the graphical handles of all the
            % objects.

            for i = 1:numel(this.Entities)
                % Get the renderer frome the entity
                renderer_i = this.Entities(i).Renderer; %("MeshRenderer");
                if isempty(renderer_i)
                    continue;
                end
                % Entities
                renderer_i.Update(ax);
            end
            drawnow;
        end
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