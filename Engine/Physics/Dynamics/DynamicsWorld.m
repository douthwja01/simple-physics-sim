
classdef DynamicsWorld < CollisionWorld
    % Physics world primitive responsible for managing the dynamic
    % properties of the simulation.

    properties
        SubSteps = 5;
        Integrator = VerletIntegrator();    % Numerical integrators
    end    
    properties (SetAccess = private)
        EnableSubStepping = true;
        Gravity = [0;0;-9.81];
        Bodies = RigidBody.empty;
    end
    % Main
    methods
        function [this] = DynamicsWorld(world)
            % DYNAMICSWORLD - Construct an instance of the dynamics world
            % object.
            
            % Call the parent
            [this] = this@CollisionWorld(world);
        end
        % Get/sets
        function set.Integrator(this,int)
            assert(isa(int,"Integrator"),"Expecting a valid integrator.");
            this.Integrator = int;
        end
        function set.SubSteps(this,steps)
            assert(mod(steps,1) == 0 && steps > 0,"Expecting an integer number of substeps.")
            this.SubSteps = steps;
        end
    end

    % Interactions
    methods        
        % High-level
        function [this] = Initialise(this)
            % Initialise the dynamic world.

            % Validate substepping
            if this.SubSteps > 1
                this.EnableSubStepping = true;
            else
                this.EnableSubStepping = false;
            end
            assert(~isempty(this.Integrator),"Require a valid numerical integration method.");

        end
        function [this] = Step(this,dt)
            % This function steps the physics simulation.

            % Sanity check
            assert(isnumeric(dt),"Expecting a valid time step.");
            
            % Step (or substep) the world
            if this.EnableSubStepping
                subTimeDelta = dt/this.SubSteps;
                for s = 1:this.SubSteps
                    this.SubStep(subTimeDelta);
                end
            else
                this.SubStep(this.TimeDelta);
            end
        end
        % Add/remove rigidbodies
        function [this] = AddRigidBody(this,body)
            % Sanity check
            assert(isa(body,"RigidBody"),"Expecting a valid RigidBody object.");
            % Add a given body to the list of objects.
            this.Bodies = vertcat(this.Bodies,body);
        end
        function [this] = RemoveRigidBody(this,body)
            % Delete a given body from the physics world.
            if isnumeric(body)
                % Temporary index
                vec = 1:1:numel(this.Bodies);     
                % Remove the object 
                selectionLogicals = vec ~= body;
            else
                assert(isa(body,"RigidBody"),"Expecting a valid 'RigidBody' object.");
                % Remove a given solver from the array of collisions solvers.
                selectionLogicals = this.Bodies ~= body;
            end
            % Remove the body
            this.Bodies = this.Bodies(selectionLogicals);
        end
        % World dynamics
        function [this] = SetGravity(this,g)
            assert(IsColumn(g,3),"Expecting a global gravity vector [3x1].");
            this.Gravity = g;
        end
    end
    % Utilities
    methods(Access = private)
        function [this] = SubStep(this,dt)
            % This function computes each physics substep.

            % The step procedure
            this.ApplyGravity();
            % Solve the collisions
            this.FindResolveCollisions(dt);
            % Update the changes of everything
            this.MoveObjects(dt);
        end        
        function [this] = ApplyGravity(this)
            % This function applies gravity to all particles

            for i = 1:numel(this.Bodies)
                body_i = this.Bodies(i);
                % If this body is not effected by gravity
                if ~body_i.IsDynamic
                    continue;
                end
                % Apply gravity
                body_i.Accelerate(this.Gravity);
            end
        end
        function [this] = MoveObjects(this,dt)
            % Update the physics properties of the world and recalculate
            % the acceleration properties of all objects based on their
            % instantaneous kinematic configurations.
            
            % Update rigidbodies (accelerations)
            for i = 1:numel(this.Bodies)
                this.Bodies(i).Update();
            end
            % Extract only the transform
            bodyTransforms = [this.Bodies.Transform];
            % Use the integrator components to integrate
            this.Integrator.Integrate(bodyTransforms,dt);
        end
    end
end