
classdef DynamicsWorld < CollisionWorld
    % Physics world primitive responsible for managing the dynamic
    % properties of the simulation.

    properties (Constant)
        Name = "Physics calculation module.";
    end
    properties
        Dynamics = RNEDynamics();           % Dynamics calculation approach
        Integrator = VerletIntegrator();    % Numerical integration approach
        EnableSubStepping = true;
        SubSteps = 5;
    end    
    properties (SetAccess = private)
        Gravity = [0;0;-9.81];
        Bodies = RigidBody.empty;
    end
    
    %% Main
    methods
        function [this] = DynamicsWorld(worldSize)
            % DYNAMICSWORLD - Construct an instance of the dynamics world
            % object.
            
            % Input check
            if nargin < 1
                worldSize = 10;
            end

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

    %% Interactions
    methods        
        % High-level
        function [this] = Initialise(this)
            % Initialise the dynamic world.

            % Validate substepping
            if this.EnableSubStepping && this.SubSteps == 1
                error("Cannot enable substepping, substeps must be greater than one.");
            end

            % Configuration sanity check
            assert(~isempty(this.Dynamics),"Cannot initialise, no dynamics element assigned.");
            assert(~isempty(this.Integrator),"Cannot initialise, no valid numerical integration method assigned.");
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
                this.SubStep(dt);
            end
        end
        % Add/remove rigidbodies
        function [this] = AddRigidBody(this,body)
            % Sanity check
            if isempty(body)
                return
            end
            assert(isa(body,"RigidBody"),"Expecting a valid RigidBody object.");
            % Add a given body to the list of objects.
            this.Bodies = vertcat(this.Bodies,body);
        end
        function [this] = RemoveRigidBody(this,body)
            % Delete a given body from the physics world (by id or body).

            % Sanity check
            if isempty(body)
                return
            end
            % Delete a given body from the physics world.
            % Delete mode
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

    %% Internals
    methods(Access = private)
        function [this] = SubStep(this,dt)
            % This function computes each physics substep.
            
            % The step procedure
            this.ApplyGravity();
            % Solve the collisions
            this.FindResolveCollisions(dt);
            % Update rigidbodies (accelerations)
            for i = 1:numel(this.Bodies)
                this.Bodies(i).Update();
            end
            % Extract only the transform
            bodyTransforms = [this.Bodies.Transform];
            % Use the integrator components to integrate
            this.Integrator.Integrate(bodyTransforms,dt);
    
            % Update statics (poses) (will change through integration)
            this.UpdateTransforms();
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
    % Internals
    methods(Access = private)
        function [this] = SubStep(this,dt)
            % This function computes each physics substep.

            % Update statics (poses) (will change through integration)
            this.World.UpdateTransforms();
            
            % === Find/Solve the collisions
            this.FindResolveCollisions(dt);

            % Isolate non-statics
            allTransforms = this.World.Root.Transform.ListChildrenOfType("Transform");
%             bodyTransforms = [this.Bodies.Transform];
%             kinematicLogicals = ~[bodyTransforms.IsStatic];
%             kinematicBodies = bodyTransforms(kinematicLogicals);
%             % Extract the kinematics
%             kinematicBodies = bodyTransforms(kinematicLogicals);
            
            % === Update the changes of everything ===
            this.Dynamics.Compute(allTransforms);
            % === Integrate the state change ===
            this.Integrator.Integrate(allTransforms,dt);
            %             this.Integrator.Integrate(kinematicBodies,dt);
        end    
    end
end