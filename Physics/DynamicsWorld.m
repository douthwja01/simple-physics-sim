
classdef DynamicsWorld < CollisionWorld
    % Physics world primitive responsible for managing the dynamic
    % properties of the simulation.
    
    properties (SetAccess = private)
        % Timing
        EnableSubStepping = true;
        SubSteps = 5;
        Gravity = [0;0;-9.81];
        Links = SimLink.empty;
        Bodies = RigidBody.empty;
        % Numerical integrators
        Integrator = VerletIntegrator();
    end
    % Main
    methods
        function [this] = DynamicsWorld(varargin)
            % Physics world constructor.
            
            % Collision world
            this = this@CollisionWorld(varargin{:});
        end
        % Get/sets
        function set.Integrator(this,int)
            assert(isa(int,"Integrator"),"Expecting a valid integrator.");
            this.Integrator = int;
        end
    end

    methods        
        % High-level
        function [this] = Initialise(this,subSteps)
            % Initialise the dynamic world.

            % Sanity check
            assert(mod(subSteps,1) == 0 && subSteps > 0,"Expecting an integer number of substeps.")

            if subSteps > 1
                this.EnableSubStepping = true;
            else
                this.EnableSubStepping = false;
            end
            this.SubSteps = subSteps;
        end
        function [this] = Step(this,dt)
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
            
            if this.EnableSubStepping
                for s = 1:this.SubSteps
                    this.ResolveCollisions(subTimeDelta);
                end
            else
                this.ResolveCollisions(dt);
            end
        end
        % Add/remove rigidbodies
        function [this] = AddRigidBody(this,body)
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
        % Add/remove Links
        function [this] = AddLink(this,objectA,objectB)
            assert(isa(objectA,"SimObject"),"Expecting a first valid SimObject.");
            assert(isa(objectB,"SimObject"),"Expecting a second valid SimObject.");
            % Create a new link object
            newLink = Link(objectA,objectB);
            this.Links = vertcat(this.Links,newLink);
        end
        function [this] = DeleteLink(this,objectA,objectB)
            % Delete a link between the objects.
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
            % The step procedure
            this.ApplyGravity();
            % Update the changes of everything
            this.UpdateAccelerations();
            % Integrate at the end of the step
            this.Integrate(dt);
        end        
        function [this] = Integrate(this,dt)
            % Use the integrator components to integrate
            this.Integrator.Integrate(this.Bodies,dt);
        end

        function [this] = ApplyGravity(this)
            % This function applies gravity to all particles

            for i = 1:numel(this.Bodies)
                body_i = this.Bodies(i);
                % If this body is not effected by gravity
                if (~body_i.IsDynamic)
                    continue;
                end
                % Apply gravity
                body_i.Accelerate(this.Gravity);
            end
        end
        function [this] = UpdateAccelerations(this)
            % Update the physics properties of the world and recalculate
            % the acceleration properties of all objects based on their
            % instantaneous kinematic configurations.
            
            % Update rigidbodies (accelerations)
            for i = 1:numel(this.Bodies)
                this.Bodies(i).Update();
            end
        end

        % Constraints
        function [this] = ApplyLinks(this)
            % This function applys the set of link constraints to the
            % system.
            for i = 1:numel(this.Links)
                this.Links(i).Apply();
            end
        end
        function [this] = ApplyWorldConstraint(this)
            % This function applies the world-constraint of a fixed sphere.

            globeCenter = [0;0;10];
            globeRadius = 10;

            for i = 1:numel(this.Bodies)
                object_i = this.Bodies(i);

                transform_i = object_i.Entity.GetElement("Transform");
                collider_i = object_i.Entity.GetElement("Collider");
                
                v = transform_i.position - globeCenter;
                distance = norm(v);

                % Check the constraint distance
                delta = globeRadius - collider_i.Radius;
                if distance <= delta
                    continue;
                end
                unit_v = v/distance;
                % Position
                transform_i.position = globeCenter + delta*unit_v;
            end
        end
    end
end