
classdef DynamicsWorld < CollisionWorld
    % Physics world primitive responsible for managing the dynamic
    % properties of the simulation.

    properties
        Dynamics = RNEDynamics();           % Dynamics calculation approach
        Integrator = VerletIntegrator();    % Numerical integration approach
        SubSteps = 5;
    end    
    properties (SetAccess = private)
        EnableSubStepping = true;
        Gravity = [0;0;-9.81];
        Bodies = RigidBody.empty;
    end
    % Main
    methods
        function [this] = DynamicsWorld(worldSize)
            % DYNAMICSWORLD - Construct an instance of the dynamics world
            % object.
            
            % Call the parent
            [this] = this@CollisionWorld(worldSize);
        end
        % Get/sets
        function set.Dynamics(this,dyn)
            assert(isa(dyn,"DynamicsSolver"),"Expecting a valid dynamics-solver.");
            this.Dynamics = dyn;
        end
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

            % Configuration sanity check
            if isempty(this.Dynamics)
                error("Cannot initialise, no dynamics element assigned.");
            end
            if isempty(this.Integrator)
                error("Cannot initialise, no integrator element assigned.");
            end
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
                this.Bodies = this.Bodies(vec ~= body);
            else
                assert(isa(body,"RigidBody"),"Expecting a valid 'RigidBody' object.");
                % Remove a given solver from the array of collisions solvers.
                this.Bodies = this.Bodies(this.Bodies ~= body);
            end
        end
        % World dynamics
        function [this] = SetGravity(this,g)
            assert(IsColumn(g,3),"Expecting a global gravity vector [3x1].");
            this.Gravity = g;
        end
    end
    % Internals
    methods(Access = private)
        function [this] = SubStep(this,dt)
            % This function computes each physics substep.

            % The step procedure
            %this.ApplyGravity();
            % Solve the collisions
            this.FindResolveCollisions(dt);

            % Isolate non-statics
            bodyTransforms = [this.Bodies.Transform];
            kinematicLogicals = ~[bodyTransforms.IsStatic];
            % Extract the kinematics
            kinematicBodies = bodyTransforms(kinematicLogicals);

            % Update the changes of everything
            this.Dynamics.Compute(kinematicBodies);
            % Use the integrator components to integrate
            this.Integrator.Integrate(kinematicBodies,dt);
        end    
    end
end