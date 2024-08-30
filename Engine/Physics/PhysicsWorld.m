classdef PhysicsWorld < CollisionWorld
    % Physics world primitive responsible for managing the dynamic
    % properties of the simulation.

    properties
        EnableSubStepping = true;
        SubSteps = 5;
        Gravity = [0;0;-9.81];
        % Solvers  
        Dynamics = NieveDynamics();                     % Dynamics (velocity, forces, acceleration etc) approach
        ConstraintSolver = GlobalNumericSolver();       % Resolve constraint forces.
        OdeSolver = EulerSolver();                     % Numerical integration approach (x0 -> x1)
    end    
    properties (SetAccess = private)
        Bodies = RigidBody.empty;
        State  = WorldState.empty;
    end
    
    %% Main
    methods
        function [this] = PhysicsWorld(worldSize)
            % DYNAMICSWORLD - Construct an instance of the dynamics world
            % object.
            
            % Input check
            if nargin < 1
                worldSize = 10;
            end

            % Call the parent
            [this] = this@CollisionWorld(worldSize);

            % Create the world-state object
            this.State = WorldState(numel(this.Bodies));
        end
        % Get/sets
        function set.Dynamics(this,dyn)
            assert(isa(dyn,"DynamicsModule"),"Expecting a valid dynamics module.");
            this.Dynamics = dyn;
        end
        function set.ConstraintSolver(this,solver)
            assert(isa(solver,"GlobalConstraintSolver"),"Expecting a valid global constraint solver.");
            this.ConstraintSolver = solver;
        end
        function set.OdeSolver(this,int)
            assert(isa(int,"OdeSolver"),"Expecting a valid ODE solver module.");
            this.OdeSolver = int;
        end
        function set.SubSteps(this,steps)
            assert(mod(steps,1) == 0 && steps > 0,"Expecting an integer number of substeps.")
            this.SubSteps = steps;
        end
        function set.Gravity(this,g)
            assert(IsColumn(g,3),"Expecting a global gravity vector [3x1].");
            this.Gravity = g;
        end
    end

    %% Interactions
    methods        
        % High-level
        function [this] = Initialise(this)
            % Initialise the dynamic world.

            % Validate substepping
            if this.EnableSubStepping
                assert(this.SubSteps > 1,"Substeps must be greater than one to enable substepping.");
            end

            % Configuration sanity check
            assert(~isempty(this.Dynamics),"Cannot initialise, no valid dynamics module assigned.");
            assert(~isempty(this.ConstraintSolver),"Cannot initialise, no valid constraint solver assigned.");
            assert(~isempty(this.OdeSolver),"Cannot initialise, no valid numerical integration method assigned.");
        
            % Initialise the modules
            this.Dynamics.Initialise(this);
            this.ConstraintSolver.Initialise(this.Bodies);
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

            this.State.Resize(numel(this.Bodies));
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
            % Resize the world-state vector
            this.State.Resize(numel(this.Bodies));
        end
    end

    %% Internals
    methods(Access = protected)
        function [this] = SubStep(this,dt)
            % This function computes each physics substep.

            % == Compute motion updates ==
            this.Dynamics.Update(dt,[this.Bodies]);

            % == Find/solve the collisions == 
            [constraints] = this.FindCollisions();

            % == Solve the contraints == 
            % (needs to be collision and dynamic)
            this.ConstraintSolver.Solve(constraints,dt);

            % == Integrate the new motion properties == 
            % Update .State
            this.UpdateStateFromBodies(this.State,this.Bodies);
            % Integrate
            this.OdeSolver.Start(this.State,dt);
            this.State = this.OdeSolver.Solve();
            this.OdeSolver.End();
            % Update .Bodies
            this.UpdateBodiesFromState(this.State,this.Bodies);
        end 
    end
    methods (Static)
        function [state]  = UpdateStateFromBodies(state,bodies)
            % This function updates the state vector from the bodies and
            % transforms.

            % Sanity check
            assert(state.NumberOfObjects == numel(bodies),"Number of bodies misaligned.");
        
            for i = 1:state.NumberOfObjects
                % Data
                data_i = state.Objects(i);
                body_i = bodies(i);

                data_i.IsStatic = body_i.IsStatic;
                % Pose
                data_i.SO3 = body_i.Transform.Local; %Inertial;
                % Velocities
                data_i.LinearVelocity = body_i.LinearVelocity;
                data_i.AngularVelocity = body_i.AngularVelocity;
                % Accelerations
                data_i.LinearAcceleration = body_i.LinearAcceleration;
                data_i.AngularAcceleration = body_i.AngularAcceleration;
                % Momentum
                data_i.LinearMomentum = body_i.LinearMomentum;
                data_i.AngularMomentum = body_i.AngularMomentum;

                state.Objects(i) = data_i;
            end
        end
        function [bodies] = UpdateBodiesFromState(state,bodies)
            % This function updates bodies from the state structure.
            
            % Sanity check
            assert(state.NumberOfObjects == numel(bodies),"Number of bodies misaligned.");
        
            for i = 1:state.NumberOfObjects
                % Data
                data_i = state.Objects(i);
                body_i = bodies(i);
                % Pose
                body_i.Transform.Local = data_i.SO3; %Inertial
                % Velocities
                body_i.LinearVelocity = data_i.LinearVelocity;
                body_i.AngularVelocity = data_i.AngularVelocity;
                % Accelerations
                body_i.LinearAcceleration = data_i.LinearAcceleration;
                body_i.AngularAcceleration = data_i.AngularAcceleration;
                % Momentum
                body_i.LinearMomentum = data_i.LinearMomentum;
                body_i.AngularMomentum = data_i.AngularMomentum;
            end
        end
    end
%% 
end